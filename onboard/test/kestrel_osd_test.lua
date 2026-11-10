-- kestrel_osd.lua against a mock of ArduPilot's scripting bindings.
--   lua5.3 kestrel_osd_test.lua path/to/kestrel_osd.lua
--
-- What it pins: the mavlink_message_t layouts (old 291-byte, new >= 298-byte,
-- as in ArduPilot's modules/MAVLink/mavlink_msgs.lua), that only `len` payload
-- bytes are read (the rest of the structure is filled with junk here), the
-- sysid/compid filter, show-on-change, the refresh, and LOST on a silent Pi.

local script = arg[1] or "kestrel_osd.lua"
local fails = 0
local function check(cond, what)
  if not cond then fails = fails + 1; print("FAIL " .. what) end
end

-- ---- the mock -------------------------------------------------------------
local now, queue, sent, registered = 0, {}, {}, {}
millis = function() return { toint = function() return now end } end
gcs = { send_text = function(_, sev, t) sent[#sent + 1] = { sev = sev, text = t } end }
mavlink = {
  init = function() end,
  register_rx_msgid = function(_, id) registered[id] = true end,
  receive_chan = function()
    local m = table.remove(queue, 1)
    if m then return m, 0, now end
  end,
}

-- A mavlink_message_t: header, `len` payload bytes, then JUNK to full size.
local function msg(wide, sysid, compid, msgid, payload)
  local len = #payload
  local h
  if wide then
    h = string.pack("<HBBBBBI4BI3", 0, 0xFD, len, 0, 0, 7, sysid, compid, msgid)
  else
    h = string.pack("<HBBBBBBBI3", 0, 0xFD, len, 0, 0, 7, sysid, compid, msgid)
  end
  local s = h .. payload
  local size = wide and 298 or 291
  return s .. string.rep("X", size - #s)
end
local function statustext(wide, t, sysid, compid)
  -- On the wire trailing NULs are truncated, so len stops at the text.
  return msg(wide, sysid or 255, compid or 191, 253, string.char(6) .. t)
end
local function heartbeat(wide)
  return msg(wide, 255, 191, 0, string.pack("<I4BBBBB", 0, 18, 8, 0, 4, 3))
end

local loop, period = dofile(script)
check(registered[0] and registered[253], "registers HEARTBEAT and STATUSTEXT")
check(period == 100, "runs every 100 ms")
local function run(ms)
  local stop = now + ms
  while now < stop do
    loop, period = loop()
    now = now + period
  end
end

for _, wide in ipairs({ false, true }) do
  local L = wide and "new layout" or "old layout"
  loop, period = dofile(script)               -- a fresh FC boot per layout
  sent = {}
  queue = { heartbeat(wide), statustext(wide, "SCRIPT LIVE VEL GO DOOR") }
  run(100)
  check(#sent == 1 and sent[1].text == "K SCRIPT LIVE VEL GO DOOR" and sent[1].sev == 6,
        L .. ": the line is shown, junk past len ignored (" .. tostring(sent[1] and sent[1].text) .. ")")

  -- The same line again shows nothing new; a heartbeat keeps it alive.
  queue = { heartbeat(wide), statustext(wide, "SCRIPT LIVE VEL GO DOOR") }
  run(1000)
  check(#sent == 1, L .. ": unchanged line not repeated")

  -- Another sender's text is not ours: a GCS (190) or another system.
  queue = { statustext(wide, "GCS SAYS HI", 255, 190), statustext(wide, "OTHER", 1, 1) }
  run(100)
  check(#sent == 1, L .. ": foreign STATUSTEXT ignored")

  -- A change goes up at once.
  queue = { heartbeat(wide), statustext(wide, "SCRIPT LIVE VEL GO LAND") }
  run(100)
  check(#sent == 2 and sent[2].text == "K SCRIPT LIVE VEL GO LAND", L .. ": change shown at once")

  -- Refresh before OSD_MSG_TIME (10 s) hides it, while heartbeats continue.
  for _ = 1, 9 do queue[#queue + 1] = heartbeat(wide); run(1000) end
  check(#sent == 3 and sent[3].text == "K SCRIPT LIVE VEL GO LAND", L .. ": refreshed after 8 s")

  -- The Pi goes silent: LOST, as a warning, and it stays up.
  run(3500)
  check(sent[#sent].text == "K LOST" and sent[#sent].sev == 4, L .. ": LOST after 3 s of silence")
  local n = #sent
  run(9000)
  check(#sent == n + 1 and sent[#sent].text == "K LOST", L .. ": LOST kept on screen")

  -- It comes back: the current line is shown again.
  queue = { heartbeat(wide), statustext(wide, "FLY DRY OFF") }
  run(100)
  check(sent[#sent].text == "K FLY DRY OFF", L .. ": back from LOST with the new line")

  -- A full 50-character line has no NUL at all.
  local fifty = string.rep("A", 50)
  queue = { heartbeat(wide), statustext(wide, fifty) }
  run(100)
  check(sent[#sent].text == "K " .. fifty, L .. ": 50 characters, no terminator")

  queue = {}
end

print(fails == 0 and "kestrel_osd.lua: all checks passed" or ("kestrel_osd.lua: " .. fails .. " FAILED"))
os.exit(fails == 0 and 0 or 1)
