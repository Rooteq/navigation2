# Shape of one collision monitor / collision detector polygon.

# Polygon name, as listed in the node's "polygons" parameter
string name

# Whether the polygon is checked. A disabled polygon may leave "polygon" empty:
# it then keeps the vertices it had.
bool enabled true

# Vertices in the node's base_frame_id (x forward, y left), in drawing order.
# At least 4 vertices; edges must not cross each other. z is ignored.
geometry_msgs/Polygon polygon
