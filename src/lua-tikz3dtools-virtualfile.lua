--- Transient Lua-backed input files for large generated TikZ streams.
---
--- The rendered drawing stays in Lua memory and TeX consumes it one line at a
--- time through LuaTeX's file callbacks.  No file is written to disk.  This is
--- especially useful for animations, where repeatedly pushing thousands of
--- generated paths directly into TeX's input can accumulate substantial TeX
--- state over a long job.
local VirtualFile = {}
VirtualFile.__index = VirtualFile

local Recovery = require "lua-tikz3dtools-recovery"

local insert = table.insert
local input = token.new(0, token.command_id("input"))
local bgroup = token.new(utf8.codepoint("{"), token.command_id("left_brace"))
local egroup = token.new(utf8.codepoint("}"), token.command_id("right_brace"))

local file_contents = {}
local number_of_files = 0
local callback_depth = 0
local displaced_find_callback
local displaced_open_callback

local FIND_CALLBACK = "lua-tikz3dtools.virtualfile.find_read_file"
local OPEN_CALLBACK = "lua-tikz3dtools.virtualfile.open_read_file"
local NAME_PREFIX = "lua-tikz3dtools-virtual-"

local function filename_for(index)
    return NAME_PREFIX .. tostring(index) .. ".tex"
end

local function default_find_read_file(_, filename)
    local handle = io.open(filename, "rb")
    if handle then
        handle:close()
        return filename
    end
    return kpse.find_file(filename, "tex")
end

local function default_open_read_file(filename)
    local handle = io.open(filename, "rb")
    if not handle then
        return nil
    end

    return {
        reader = function()
            return handle:read("*l")
        end,
        close = function()
            handle:close()
        end,
    }
end

local function find_read_file(...)
    local args = {...}
    local filename = args[2]
    if file_contents[filename] then
        return filename
    end
    if displaced_find_callback then
        return displaced_find_callback.func(...)
    end
    return default_find_read_file(...)
end

local function open_read_file(filename, ...)
    local contents = file_contents[filename]
    if not contents then
        if displaced_open_callback then
            return displaced_open_callback.func(filename, ...)
        end
        return default_open_read_file(filename)
    end

    -- Remove the registry reference immediately.  The environment returned to
    -- LuaTeX owns the table from here until EOF, so a virtual file is naturally
    -- one-shot and can be garbage-collected as soon as TeX finishes reading it.
    file_contents[filename] = nil

    return {
        contents = contents,
        cursor = 0,
        reader = function(self)
            self.cursor = self.cursor + 1
            return self.contents[self.cursor]
        end,
        close = function(self)
            self.contents = nil
        end,
    }
end

local function displace_exclusive_callback(name, own_description)
    local descriptions = luatexbase.callback_descriptions(name)
    local displaced

    for _, description in pairs(descriptions) do
        if description ~= own_description then
            local func, removed_description = luatexbase.remove_from_callback(name, description)
            displaced = {
                func = func,
                description = removed_description,
            }
            break
        end
    end

    return displaced
end

local function restore_exclusive_callback(name, displaced)
    if displaced then
        luatexbase.add_to_callback(name, displaced.func, displaced.description)
    end
end

local function add_callbacks()
    if callback_depth > 0 then
        callback_depth = callback_depth + 1
        return
    end

    -- find_read_file/open_read_file are exclusive luatexbase callbacks.  A
    -- package-safe virtual input therefore cannot simply register a second
    -- callback.  Temporarily displace the current owner, delegate all non-
    -- virtual reads to it, then restore it immediately after our \input.
    displaced_find_callback = displace_exclusive_callback("find_read_file", FIND_CALLBACK)
    displaced_open_callback = displace_exclusive_callback("open_read_file", OPEN_CALLBACK)

    local find_installed = false
    local open_installed = false
    local ok, err = pcall(function()
        luatexbase.add_to_callback("find_read_file", find_read_file, FIND_CALLBACK)
        find_installed = true
        luatexbase.add_to_callback("open_read_file", open_read_file, OPEN_CALLBACK)
        open_installed = true
    end)

    if not ok then
        if open_installed and luatexbase.in_callback("open_read_file", OPEN_CALLBACK) then
            luatexbase.remove_from_callback("open_read_file", OPEN_CALLBACK)
        end
        if find_installed and luatexbase.in_callback("find_read_file", FIND_CALLBACK) then
            luatexbase.remove_from_callback("find_read_file", FIND_CALLBACK)
        end
        restore_exclusive_callback("open_read_file", displaced_open_callback)
        restore_exclusive_callback("find_read_file", displaced_find_callback)
        displaced_find_callback = nil
        displaced_open_callback = nil
        error(err, 0)
    end

    callback_depth = 1
end

local function remove_callbacks()
    if callback_depth <= 0 then
        Recovery.log_once(
            "virtualfile-underflow",
            "virtual input callback cleanup recovered",
            "callback stack was already empty"
        )
        callback_depth = 0
        return
    end
    callback_depth = callback_depth - 1
    if callback_depth > 0 then
        return
    end

    local old_open = displaced_open_callback
    local old_find = displaced_find_callback
    displaced_find_callback = nil
    displaced_open_callback = nil

    local ok, err = pcall(function()
        if luatexbase.in_callback("open_read_file", OPEN_CALLBACK) then
            luatexbase.remove_from_callback("open_read_file", OPEN_CALLBACK)
        end
        if luatexbase.in_callback("find_read_file", FIND_CALLBACK) then
            luatexbase.remove_from_callback("find_read_file", FIND_CALLBACK)
        end
        restore_exclusive_callback("open_read_file", old_open)
        restore_exclusive_callback("find_read_file", old_find)
    end)
    if not ok then
        Recovery.log(
            "virtual input callback cleanup recovered",
            tostring(err) .. "; continuing without aborting the document"
        )
    end
end

local function anonymous_tex_cmd(func)
    local index = luatexbase.new_luafunction()
    lua.get_functions_table()[index] = func
    return token.new(index, token.command_id("lua_expandable_call"))
end

-- Callback installation is attempted synchronously when :input() is called so
-- failure can fall back to direct TeX streaming instead of leaving an unresolved
-- virtual \input in TeX's mouth.  Cleanup remains a queued token after the input.
local pop_callbacks = anonymous_tex_cmd(remove_callbacks)

--- Create one transient virtual TeX file.
--- @return table file object with :write(line), :input(), and :discard().
function VirtualFile.new()
    number_of_files = number_of_files + 1
    local filename = filename_for(number_of_files)
    local contents = {}
    file_contents[filename] = contents

    local state = "open"
    local file = {}

    function file:write(line)
        if state ~= "open" then
            Recovery.log_once(
                "virtualfile-write-closed",
                "generated output line omitted",
                "virtual file was already closed"
            )
            return false
        end
        if type(line) ~= "string" then
            Recovery.log_once(
                "virtualfile-write-type",
                "generated output line omitted",
                "output was not a string"
            )
            return false
        end
        insert(contents, line)
        return true
    end

    function file:input()
        if state ~= "open" then
            Recovery.log_once(
                "virtualfile-input-closed",
                "virtual input ignored",
                "virtual file was already consumed or discarded"
            )
            return false
        end

        local ok, err = pcall(add_callbacks)
        if not ok then
            -- Nothing on disk exists under filename, so never queue \input after
            -- callback installation failed.  Stream the retained lines directly
            -- as the last-resort transport; this may be less memory-efficient but
            -- keeps the document alive.
            Recovery.log(
                "virtual input recovered",
                tostring(err) .. "; using direct TeX streaming"
            )
            file_contents[filename] = nil
            state = "streamed"
            for _, line in ipairs(contents or {}) do tex.sprint(line) end
            contents = nil
            return true
        end

        state = "queued"

        -- Catcode table -2 means token objects pass through exactly while the
        -- filename string is tokenized in the normal current context.
        tex.sprint(-2, {
            input, bgroup, filename, egroup,
            pop_callbacks,
        })

        -- open_read_file() transfers ownership of the backing table to LuaTeX.
        contents = nil
        return true
    end

    function file:discard()
        if state == "open" then
            file_contents[filename] = nil
            contents = nil
            state = "discarded"
        end
    end

    function file:name()
        return filename
    end

    return file
end

return VirtualFile
