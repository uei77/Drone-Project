--[[
Situation Selector flight-controller side (ArduCopter Lua script) - REFERENCE, NOT YET RUN IN SITL.

Wiring/params:
  SCR_ENABLE = 1, SERIALx_PROTOCOL = 28 (Scripting), SERIALx_BAUD = 115
  ESP32 GPIO14 (TX) -> FC RX, ESP32 GPIO15 (RX) <- FC TX, common GND.

Safety model: the ESP32 only PROPOSES. This script decides, limits speed/duration,
and a dead ESP32 can never stop the mission (it can only fail to start a reaction).

Protocol: see esp32_camera_scripts/main/proto.h
]]

local MODE_AUTO, MODE_GUIDED, MODE_RTL, MODE_LAND = 3, 4, 6, 9
local PARAM_KEY = 72
assert(param:add_table(PARAM_KEY, "SSV_", 8), "SSV_ param table")
param:add_param(PARAM_KEY, 1, "ENABLE", 1)      -- 0 off, 1 react to events
param:add_param(PARAM_KEY, 2, "SPEED", 0.5)     -- m/s, max speed of one $ACT step
param:add_param(PARAM_KEY, 3, "MIN_ALT", 5)     -- m, no reactions below this
param:add_param(PARAM_KEY, 4, "LAND_OK", 0)     -- 1 = "land" from vision is allowed
param:add_param(PARAM_KEY, 5, "FAIL_RTL", 0)    -- 1 = RTL instead of AUTO on timeouts
param:add_param(PARAM_KEY, 6, "MAX_PAUSE", 120) -- s, longest vision pause (centre + guided descent takes about a minute)
param:add_param(PARAM_KEY, 7, "VMODE", 1)       -- vision mode sent to ESP32: 0 off, 1 shadow, 2 active

local P_ENABLE, P_SPEED = Parameter("SSV_ENABLE"), Parameter("SSV_SPEED")
local P_MIN_ALT, P_LAND_OK = Parameter("SSV_MIN_ALT"), Parameter("SSV_LAND_OK")
local P_FAIL_RTL, P_MAX_PAUSE = Parameter("SSV_FAIL_RTL"), Parameter("SSV_MAX_PAUSE")
local P_VMODE = Parameter("SSV_VMODE")

local port = serial:find_serial(0)
assert(port, "Situation Selector: no scripting serial port")
port:begin(115200)
port:set_flow_control(0)

local rx = ""
local hb_ok, hb_ms = false, 0
local paused, pause_seq, pause_start, last_cmd = false, -1, 0, 0
local act_until, act_vel = 0, Vector3f()
local last_tel, last_vmode, last_vmode_ms = 0, -1, 0

local function now() return millis():tofloat() end

local function checksum(s)
  local cs = 0
  for i = 1, #s do cs = cs ~ s:byte(i) end
  return cs
end

local function send(payload)
  local line = string.format("$%s*%02X\n", payload, checksum(payload))
  for i = 1, #line do port:write(line:byte(i)) end
end

local function end_pause(reason)
  if not paused then return end
  paused = false
  act_until = 0
  gcs:send_text(6, "SSV: resume (" .. reason .. ")")
  if vehicle:get_mode() == MODE_GUIDED then
    if P_FAIL_RTL:get() == 1 and reason ~= "done" then vehicle:set_mode(MODE_RTL)
    else vehicle:set_mode(MODE_AUTO) end
  end
end

local function alt_m()
  local d = ahrs:get_relative_position_D_home()
  return d and -d or 0
end

local function on_evt(f)
  local seq = tonumber(f[2])
  if not seq then return end
  if f[3] == "done" then
    if paused and seq == pause_seq then end_pause("done") end
    return
  end
  if paused and seq == pause_seq then send("ACK," .. seq) return end   -- repeated EVT
  if paused or P_ENABLE:get() ~= 1 or not hb_ok then return end
  if vehicle:get_mode() ~= MODE_AUTO or not arming:is_armed() then return end
  if alt_m() < P_MIN_ALT:get() then return end
  if vehicle:set_mode(MODE_GUIDED) then          -- GUIDED with no target = hold position
    paused, pause_seq, pause_start, last_cmd = true, seq, now(), now()
    logger:write("SSVE", "Seq,Sit,Conf", "HNB", seq, f[3] or "?", tonumber(f[4]) or 0)
    gcs:send_text(5, "SSV: event " .. (f[3] or "?") .. " " .. (f[4] or "?") .. "%")
    send("ACK," .. seq)
  end
end

local DIRS = { forward = { 1, 0, 0 }, back = { -1, 0, 0 }, right = { 0, 1, 0 }, left = { 0, -1, 0 },
               up = { 0, 0, -1 }, down = { 0, 0, 1 }, hold = { 0, 0, 0 } }

local function on_act(f)
  local seq, a = tonumber(f[2]), f[3]
  if not paused or seq ~= pause_seq then return end
  last_cmd = now()
  if a == "land" then
    if P_LAND_OK:get() == 1 then vehicle:set_mode(MODE_LAND); paused = false end
    return
  end
  local d = DIRS[a]
  if not d then return end
  -- body frame (forward/right/down) -> NED
  local yaw, s = ahrs:get_yaw(), P_SPEED:get()
  local v = Vector3f()
  v:x((d[1] * math.cos(yaw) - d[2] * math.sin(yaw)) * s)
  v:y((d[1] * math.sin(yaw) + d[2] * math.cos(yaw)) * s)
  v:z(d[3] * s)
  act_vel, act_until = v, now() + 500
end

local function handle(line)
  local body, cs = line:match("^%$(.-)%*(%x%x)$")
  if not body or tonumber(cs, 16) ~= checksum(body) then return end
  local f = {}
  for x in (body .. ","):gmatch("([^,]*),") do f[#f + 1] = x end
  if f[1] == "HB" then
    hb_ok, hb_ms = (f[3] == "OK"), now()
  elseif f[1] == "EVT" then on_evt(f)
  elseif f[1] == "ACT" then on_act(f)
  elseif f[1] == "TGT" then last_cmd = now()
  elseif f[1] == "SHD" then
    logger:write("SSVS", "Sit,Conf", "NB", f[2] or "?", tonumber(f[3]) or 0)
    gcs:send_text(6, "SSV shadow: " .. (f[2] or "?") .. " " .. (f[3] or "?") .. "%")
  end
end

local function send_tel()
  local vel = ahrs:get_velocity_NED() or Vector3f()
  local gyro = ahrs:get_gyro() or Vector3f()
  local hdg = math.deg(ahrs:get_yaw()) % 360
  local brg = vehicle:get_wp_bearing_deg() or hdg
  local mode = vehicle:get_mode()
  local m = (mode == MODE_AUTO) and 1 or ((paused and mode == MODE_GUIDED) and 2) or
            ((mode == MODE_RTL or mode == MODE_LAND) and 3) or 0
  send(string.format("TEL,%d,%.2f,%.1f,%.2f,%.2f,%.2f,%.1f,%d,%d,%.1f,%d",
       math.floor(now()) % 4294967296, alt_m(), hdg, -vel:z(), vel:x(), vel:y(),
       math.deg(gyro:z()), mission:get_current_nav_index(), mission:num_commands(), brg, m))
end

local function update()
  -- receive
  local n = port:available():toint()
  for _ = 1, math.min(n, 256) do
    local c = string.char(port:read())
    if c == "\n" then handle(rx); rx = ""
    elseif c ~= "\r" then rx = (#rx < 120) and (rx .. c) or "" end
  end
  local t = now()
  if t - hb_ms > 1000 then hb_ok = false end
  if t - last_tel >= 100 then last_tel = t; send_tel() end
  local vm = P_VMODE:get()
  if vm ~= last_vmode or t - last_vmode_ms > 2000 then      -- resent: survives an ESP32 reboot
    last_vmode, last_vmode_ms = vm, t
    send(string.format("CMD,MODE,%d", vm))
  end

  if paused then
    if vehicle:get_mode() ~= MODE_GUIDED then paused = false        -- pilot took over
    elseif not hb_ok then end_pause("esp32 lost")
    elseif t - last_cmd > 3000 then end_pause("silent")
    elseif t - pause_start > P_MAX_PAUSE:get() * 1000 then end_pause("too long")
    elseif t < act_until then vehicle:set_target_velocity_NED(act_vel)
    else vehicle:set_target_velocity_NED(Vector3f()) end              -- hold
  end
  return update, 20
end

gcs:send_text(6, "Situation Selector FC script loaded")
return update, 100
