--- Deterministic static axis-aligned bounding-box hierarchy.
---
--- This module is deliberately geometry-agnostic.  It knows only about
--- numeric AABBs and caller-supplied indices.  Exact geometric decisions
--- remain the responsibility of Geometry.
--- @class AABBTree
local AABBTree = {}
AABBTree.__index = AABBTree

local DEFAULT_LEAF_SIZE = 8

local function verify_box(box, dimensions)
    assert(type(box) == "table", "AABB must be a table.")
    assert(type(box.min) == "table" and type(box.max) == "table",
        "AABB must contain min and max arrays.")
    for d = 1, dimensions do
        assert(type(box.min[d]) == "number" and type(box.max[d]) == "number",
            "AABB coordinates must be numbers.")
        assert(box.min[d] <= box.max[d], "AABB min must not exceed max.")
    end
end

local function boxes_overlap(a, b, dimensions)
    for d = 1, dimensions do
        if a.max[d] < b.min[d] or b.max[d] < a.min[d] then
            return false
        end
    end
    return true
end

local function union_box_range(entries, first, last, dimensions)
    local min_values = {}
    local max_values = {}
    for d = 1, dimensions do
        min_values[d] = math.huge
        max_values[d] = -math.huge
    end

    for i = first, last do
        local box = entries[i].box
        for d = 1, dimensions do
            if box.min[d] < min_values[d] then min_values[d] = box.min[d] end
            if box.max[d] > max_values[d] then max_values[d] = box.max[d] end
        end
    end

    return { min = min_values, max = max_values }
end

local function centroid(entry, axis)
    return (entry.box.min[axis] + entry.box.max[axis]) * 0.5
end

local function split_axis(entries, first, last, dimensions)
    local chosen_axis = 1
    local chosen_spread = -math.huge

    for d = 1, dimensions do
        local min_centroid = math.huge
        local max_centroid = -math.huge
        for i = first, last do
            local c = centroid(entries[i], d)
            if c < min_centroid then min_centroid = c end
            if c > max_centroid then max_centroid = c end
        end
        local spread = max_centroid - min_centroid
        -- Strict comparison intentionally gives lower-numbered axes tie priority.
        if spread > chosen_spread then
            chosen_spread = spread
            chosen_axis = d
        end
    end

    return chosen_axis
end

local function entry_less(a, b, axis)
    local ca = centroid(a, axis)
    local cb = centroid(b, axis)
    if ca == cb then
        return a.index < b.index
    end
    return ca < cb
end

local function swap(entries, i, j)
    entries[i], entries[j] = entries[j], entries[i]
end

local function insertion_sort_range(entries, first, last, axis)
    for i = first + 1, last do
        local value = entries[i]
        local j = i - 1
        while j >= first and entry_less(value, entries[j], axis) do
            entries[j + 1] = entries[j]
            j = j - 1
        end
        entries[j + 1] = value
    end
end

local function partition_range(entries, first, last, pivot_index, axis)
    local pivot = entries[pivot_index]
    swap(entries, pivot_index, last)
    local store = first
    for i = first, last - 1 do
        if entry_less(entries[i], pivot, axis) then
            swap(entries, store, i)
            store = store + 1
        end
    end
    swap(entries, store, last)
    return store
end

-- Deterministic median-of-medians quickselect.  This keeps median selection
-- O(n) in the worst case, so building the balanced hierarchy is O(n log n)
-- rather than depending on favorable pivot choices.
local function select_kth(entries, first, last, k, axis)
    while true do
        local count = last - first + 1
        if count <= 16 then
            insertion_sort_range(entries, first, last, axis)
            return
        end

        local median_count = 0
        local group_first = first
        while group_first <= last do
            local group_last = math.min(group_first + 4, last)
            insertion_sort_range(entries, group_first, group_last, axis)
            local group_median = math.floor((group_first + group_last) / 2)
            swap(entries, first + median_count, group_median)
            median_count = median_count + 1
            group_first = group_first + 5
        end

        local medians_last = first + median_count - 1
        local medians_k = first + math.floor((median_count - 1) / 2)
        select_kth(entries, first, medians_last, medians_k, axis)

        local pivot_index = partition_range(entries, first, last, medians_k, axis)
        if k == pivot_index then
            return
        elseif k < pivot_index then
            last = pivot_index - 1
        else
            first = pivot_index + 1
        end
    end
end

local function sorted_leaf(entries, first, last)
    local leaf_entries = {}
    for i = first, last do
        leaf_entries[#leaf_entries + 1] = entries[i]
    end
    table.sort(leaf_entries, function(a, b) return a.index < b.index end)
    return leaf_entries
end

local function build_node(entries, first, last, dimensions, leaf_size)
    local count = last - first + 1
    local node = {
        box = union_box_range(entries, first, last, dimensions),
    }

    if count <= leaf_size then
        node.entries = sorted_leaf(entries, first, last)
        return node
    end

    local axis = split_axis(entries, first, last, dimensions)
    local midpoint = math.floor((first + last) / 2)
    select_kth(entries, first, last, midpoint, axis)

    node.left = build_node(entries, first, midpoint, dimensions, leaf_size)
    node.right = build_node(entries, midpoint + 1, last, dimensions, leaf_size)
    return node
end

--- Build a static AABB hierarchy.
--- Entries must be { index = integer, box = { min = {...}, max = {...} } }.
--- @param entries table
--- @param dimensions number
--- @param options table|nil
--- @return AABBTree
function AABBTree.build(entries, dimensions, options)
    assert(type(entries) == "table", "entries must be a table.")
    assert(dimensions == 2 or dimensions == 3, "dimensions must be 2 or 3.")
    options = options or {}
    local leaf_size = options.leaf_size or DEFAULT_LEAF_SIZE
    assert(type(leaf_size) == "number" and leaf_size >= 1 and leaf_size % 1 == 0,
        "leaf_size must be a positive integer.")

    local copied = {}
    local seen_indices = {}
    for i, entry in ipairs(entries) do
        assert(type(entry) == "table", "AABB entry must be a table.")
        assert(type(entry.index) == "number" and entry.index % 1 == 0,
            "AABB entry index must be an integer.")
        assert(not seen_indices[entry.index], "AABB entry indices must be unique.")
        seen_indices[entry.index] = true
        verify_box(entry.box, dimensions)
        copied[i] = { index = entry.index, box = entry.box }
    end

    return setmetatable({
        dimensions = dimensions,
        leaf_size = leaf_size,
        root = (#copied > 0) and build_node(copied, 1, #copied, dimensions, leaf_size) or nil,
    }, AABBTree)
end

--- Test two AABBs according to this tree's dimensionality.
--- Touching boundaries count as overlap so candidate generation is conservative.
--- @param a table
--- @param b table
--- @return boolean
function AABBTree:overlaps(a, b)
    verify_box(a, self.dimensions)
    verify_box(b, self.dimensions)
    return boxes_overlap(a, b, self.dimensions)
end

--- Visit every indexed entry whose AABB overlaps query_box.
--- Traversal order is deterministic but callers that require original index order
--- should collect and sort returned indices explicitly.
--- @param query_box table
--- @param callback function
function AABBTree:query(query_box, callback)
    assert(type(callback) == "function", "callback must be a function.")
    verify_box(query_box, self.dimensions)

    local function visit(node)
        if node == nil or not boxes_overlap(node.box, query_box, self.dimensions) then
            return
        end

        if node.entries then
            for _, entry in ipairs(node.entries) do
                if boxes_overlap(entry.box, query_box, self.dimensions) then
                    callback(entry.index)
                end
            end
            return
        end

        visit(node.left)
        visit(node.right)
    end

    visit(self.root)
end

local function node_measure(node, dimensions)
    local measure = 0
    for d = 1, dimensions do
        local extent = node.box.max[d] - node.box.min[d]
        measure = measure + extent * extent
    end
    return measure
end

--- Visit every overlapping entry pair exactly once.
--- The callback receives canonical indices i < j.  Emission order is an internal
--- detail; callers should not use it to define graph traversal order.
--- @param callback function
function AABBTree:foreach_overlapping_pair(callback)
    assert(type(callback) == "function", "callback must be a function.")
    if self.root == nil then return end

    local dimensions = self.dimensions

    local function emit_entries(a_entries, b_entries, same_leaf)
        if same_leaf then
            for ai = 1, #a_entries - 1 do
                local a = a_entries[ai]
                for bi = ai + 1, #a_entries do
                    local b = a_entries[bi]
                    if boxes_overlap(a.box, b.box, dimensions) then
                        callback(a.index, b.index)
                    end
                end
            end
            return
        end

        for _, a in ipairs(a_entries) do
            for _, b in ipairs(b_entries) do
                if boxes_overlap(a.box, b.box, dimensions) then
                    if a.index < b.index then
                        callback(a.index, b.index)
                    else
                        callback(b.index, a.index)
                    end
                end
            end
        end
    end

    local visit_cross

    local function visit_same(node)
        if node.entries then
            emit_entries(node.entries, nil, true)
            return
        end

        visit_same(node.left)
        visit_cross(node.left, node.right)
        visit_same(node.right)
    end

    visit_cross = function(a, b)
        if not boxes_overlap(a.box, b.box, dimensions) then
            return
        end

        if a.entries and b.entries then
            emit_entries(a.entries, b.entries, false)
            return
        end

        if a.entries then
            visit_cross(a, b.left)
            visit_cross(a, b.right)
            return
        end

        if b.entries then
            visit_cross(a.left, b)
            visit_cross(a.right, b)
            return
        end

        -- Split the node with the larger bounding-box diagonal.  Unlike area
        -- or volume, this remains informative for line- and plane-like boxes.
        -- Ties split the left argument, making traversal deterministic.
        if node_measure(a, dimensions) >= node_measure(b, dimensions) then
            visit_cross(a.left, b)
            visit_cross(a.right, b)
        else
            visit_cross(a, b.left)
            visit_cross(a, b.right)
        end
    end

    visit_same(self.root)
end

return AABBTree
