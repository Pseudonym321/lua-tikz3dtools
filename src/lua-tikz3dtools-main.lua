Vector = require "lua-tikz3dtools-vector"
Matrix = require "lua-tikz3dtools-matrix"
Geometry = require "lua-tikz3dtools-geometry"
Scene = require "lua-tikz3dtools-scene"

Vector:_set_matrix_class(Matrix)
Matrix:_set_vector_class(Vector)
Geometry:_set_classes(Vector, Matrix)
Scene:_set_classes(Vector, Matrix, Geometry)

Scene._init_math_env()
Scene.register_commands()