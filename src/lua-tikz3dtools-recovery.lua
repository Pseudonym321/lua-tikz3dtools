--- Conservative recovery helpers.
---
--- Rendering should survive local numerical/pathological geometry failures.
--- Only significant recoveries are logged, and every message occupies one
--- physical line with the same Time:HH:MM:SS prefix as ordinary status output.
local Recovery = {}

local seen = {}
local seen_messages = {}

local function one_line(value)
    return tostring(value or "unknown failure")
        :gsub("%s+", " ")
        :gsub("^%s+", "")
        :gsub("%s+$", "")
end

local LOG_LINE_LIMIT = 76

local function write_line(context, detail)
    local prefix = ("Time:%s lua-tikz3dtools: "):format(os.date("%X"))
    local body = tostring(context or "recovered")
    local cleaned = one_line(detail)
    if cleaned ~= "" then body = body .. "; " .. cleaned end
    local room = LOG_LINE_LIMIT - #prefix
    if #body > room then
        body = body:sub(1, math.max(1, room - 3)) .. "..."
    end
    local message = prefix .. body
    if tex and tex.jobname and texio and texio.write_nl then
        texio.write_nl("term and log", message)
    else
        io.stdout:write(message, "\n")
        io.stdout:flush()
    end
end

function Recovery.log(context, detail)
    local key = tostring(context or "recovered") .. "\0" .. one_line(detail)
    if seen_messages[key] then return end
    seen_messages[key] = true
    write_line(context, detail)
end

function Recovery.log_once(key, context, detail)
    key = tostring(key or context or detail)
    if seen[key] then return end
    seen[key] = true
    Recovery.log(context, detail)
end

function Recovery.try(context, fn, fallback)
    local result = table.pack(pcall(fn))
    if not result[1] then
        Recovery.log(context, result[2])
        if type(fallback) == "function" then
            return fallback(result[2])
        end
        return fallback
    end
    return table.unpack(result, 2, result.n)
end

function Recovery.reset_once()
    seen = {}
    seen_messages = {}
end

return Recovery
