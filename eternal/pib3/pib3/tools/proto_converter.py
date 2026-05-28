#!/usr/bin/env python3
"""Convert Webots PIB proto file to URDF format for roboticstoolbox."""

import math
import re
import os

# NOTE: The canonical format uses Webots motor radians directly.

# --- Vector Math ---

def normalize(v):
    """Normalize a 3D vector."""
    norm = math.sqrt(sum(x*x for x in v))
    if norm == 0:
        return v
    return [x/norm for x in v]

def matrix_mul(R, v):
    """Multiply 3x3 matrix R by 3D vector v."""
    return [
        sum(R[i][j] * v[j] for j in range(3))
        for i in range(3)
    ]

def axis_angle_to_matrix(axis, angle):
    """Convert axis-angle representation to 3x3 rotation matrix."""
    x, y, z = normalize(axis)
    c = math.cos(angle)
    s = math.sin(angle)
    C = 1 - c
    return [
        [x*x*C + c,   x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s, y*y*C + c,   y*z*C - x*s],
        [z*x*C - y*s, z*y*C + x*s, z*z*C + c]
    ]

def matrix_mult_mat(A, B):
    """Multiply two 3x3 matrices."""
    C = [[0,0,0],[0,0,0],[0,0,0]]
    for i in range(3):
        for j in range(3):
            C[i][j] = sum(A[i][k] * B[k][j] for k in range(3))
    return C

def matrix_transpose(R):
    """Transpose a 3x3 matrix."""
    return [[R[j][i] for j in range(3)] for i in range(3)]

def matrix_to_rpy(R):
    """Convert rotation matrix to roll-pitch-yaw (XYZ Euler angles)."""
    pitch = math.atan2(-R[2][0], math.sqrt(R[0][0]**2 + R[1][0]**2))
    if math.isclose(math.cos(pitch), 0.0):
        roll = 0.0
        yaw = math.atan2(R[1][2], R[1][1])
    else:
        roll = math.atan2(R[2][1], R[2][2])
        yaw = math.atan2(R[1][0], R[0][0])
    return [roll, pitch, yaw]

def identity_matrix():
    """Return 3x3 identity matrix."""
    return [[1,0,0],[0,1,0],[0,0,1]]

def vec_add(a, b):
    """Add two 3D vectors."""
    return [x+y for x, y in zip(a, b)]

def vec_sub(a, b):
    """Subtract two 3D vectors."""
    return [x-y for x, y in zip(a, b)]

# --- Parsing ---

class Node:
    """Represents a node in the Webots proto file tree."""
    def __init__(self, type_name):
        self.type = type_name
        self.fields = {}
    def __repr__(self):
        return f"{self.type}(...)"

def tokenize(text):
    """Tokenize Webots proto file text."""
    tokens = []
    i = 0
    n = len(text)
    while i < n:
        if text[i].isspace():
            i += 1
            continue
        if text[i] == '#':
            while i < n and text[i] != '\n':
                i += 1
            continue
        if text[i] in '{}[].':
            tokens.append(text[i])
            i += 1
            continue
        if text[i] == '"':
            j = i + 1
            while j < n and text[j] != '"':
                j += 1
            tokens.append(text[i:j+1])
            i = j + 1
            continue
        j = i
        while j < n and (text[j].isalnum() or text[j] in '_-+.eE'):
            j += 1
        tokens.append(text[i:j])
        i = j
    return tokens

def parse_proto(tokens):
    """Parse proto file tokens into a Robot node tree."""
    defs = {}
    idx = 0

    def parse_value():
        nonlocal idx
        if idx >= len(tokens):
            return None
        token = tokens[idx]
        if token == 'USE':
            idx += 1
            ref = tokens[idx]
            idx += 1
            return defs.get(ref, Node(f"USE_{ref}"))
        if token == 'DEF':
            idx += 1
            name = tokens[idx]
            idx += 1
            val = parse_value()
            defs[name] = val
            return val
        if token == '[':
            idx += 1
            arr = []
            while idx < len(tokens) and tokens[idx] != ']':
                arr.append(parse_value())
            idx += 1
            return arr
        if idx + 1 < len(tokens) and tokens[idx+1] == '{':
            return parse_node()
        idx += 1
        return token

    def parse_node():
        nonlocal idx
        node = Node(tokens[idx])
        idx += 1
        if idx < len(tokens) and tokens[idx] == '{':
            idx += 1
            while idx < len(tokens) and tokens[idx] != '}':
                field = tokens[idx]
                idx += 1
                if tokens[idx] == 'IS':
                    idx += 3
                    continue
                if tokens[idx] in ['DEF', 'USE', '['] or (idx+1 < len(tokens) and tokens[idx+1] == '{'):
                    node.fields[field] = parse_value()
                else:
                    vals = []
                    while idx < len(tokens):
                        t = tokens[idx]
                        if t in ['}', '{', ']', 'DEF', 'USE', 'IS']:
                            break
                        vals.append(t)
                        idx += 1
                        if field in ['translation','anchor','centerOfMass','baseColor','axis'] and len(vals)==3:
                            break
                        if field in ['rotation'] and len(vals)==4:
                            break
                        if field in ['inertiaMatrix'] and len(vals)==6:
                            break
                        if field in ['mass','name','url','maxVelocity','minPosition','maxPosition','maxTorque','roughness','metalness'] and len(vals)==1:
                            break
                        if idx < len(tokens) and tokens[idx][0].islower() and tokens[idx] not in ['TRUE','FALSE']:
                            if not re.match(r'^-?\d', tokens[idx]):
                                break
                    node.fields[field] = vals if len(vals) > 1 else (vals[0] if vals else None)
            idx += 1
        return node

    while idx < len(tokens) and tokens[idx] != 'Robot':
        idx += 1
    return parse_node()

# --- URDF Generation ---

def get_vec3(node, field, default=None):
    """Extract a 3D vector from a node field."""
    if default is None:
        default = [0, 0, 0]
    if field in node.fields:
        v = node.fields[field]
        if isinstance(v, list):
            return [float(x) for x in v[:3]]
    return default

def get_rot(node, field):
    """Extract axis-angle rotation from a node field."""
    if field in node.fields:
        v = node.fields[field]
        if isinstance(v, list) and len(v) >= 4:
            return [float(x) for x in v[:3]], float(v[3])
    return [0, 0, 1], 0

def generate_urdf(robot_node):
    """Generate URDF from parsed Robot node.

    Returns:
        The URDF XML as a string.
    """
    lines = ['<?xml version="1.0"?>\n<robot name="pib">\n']

    if 'children' in robot_node.fields:
        for child in robot_node.fields['children']:
            if isinstance(child, Node) and child.type == 'Solid':
                process_solid_as_link(
                    lines,
                    solid_node=child,
                    link_name="base_link",
                    parent_frame_origin=[0, 0, 0],
                    parent_frame_rot=identity_matrix()
                )
                break

    lines.append('</robot>\n')
    return ''.join(lines)

def process_solid_as_link(lines, solid_node, link_name, parent_frame_origin, parent_frame_rot):
    """
    Process a Webots Solid as a URDF link.

    In Webots:
    - Solid nodes contain visual geometry via Transform/Shape children
    - Solid nodes contain HingeJoint children that connect to child Solids
    - Each Solid has its own local frame defined by translation/rotation relative to parent

    In URDF:
    - Each link has its own frame
    - Joint origin defines transform from parent link to child link
    - Visual/collision origins are relative to the link frame

    Args:
        lines: List to accumulate URDF XML lines.
        solid_node: The Webots Solid node
        link_name: URDF link name
        parent_frame_origin: Origin of parent frame in global coords
        parent_frame_rot: Rotation matrix of parent frame
    """
    # Get solid's local transform (relative to its parent)
    solid_trans = get_vec3(solid_node, 'translation')
    solid_rot_axis, solid_rot_angle = get_rot(solid_node, 'rotation')
    solid_rot = axis_angle_to_matrix(solid_rot_axis, solid_rot_angle)

    # Compute solid's global position and orientation
    solid_global_pos = vec_add(parent_frame_origin, matrix_mul(parent_frame_rot, solid_trans))
    solid_global_rot = matrix_mult_mat(parent_frame_rot, solid_rot)

    link_frame_origin = solid_global_pos
    link_frame_rot = solid_global_rot

    lines.append(f'  <link name="{link_name}">\n')

    # Process visual/collision geometry
    if 'children' in solid_node.fields:
        for c in solid_node.fields.get('children', []):
            process_visual_element(lines, c, link_frame_origin, link_frame_rot, link_frame_origin, link_frame_rot)

    # Add inertial properties if physics is defined
    if 'physics' in solid_node.fields:
        phys = solid_node.fields['physics']
        if isinstance(phys, Node):
            mass = float(phys.fields.get('mass', 1.0))
            com_local = get_vec3(phys, 'centerOfMass')
            lines.append(f'    <inertial>\n')
            lines.append(f'      <origin xyz="{com_local[0]} {com_local[1]} {com_local[2]}" rpy="0 0 0"/>\n')
            lines.append(f'      <mass value="{mass}"/>\n')
            lines.append(f'      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>\n')
            lines.append(f'    </inertial>\n')

    lines.append(f'  </link>\n')

    # Process HingeJoint children
    if 'children' in solid_node.fields:
        for c in solid_node.fields.get('children', []):
            if isinstance(c, Node) and c.type == 'HingeJoint':
                process_hinge_joint(lines, c, link_name, link_frame_origin, link_frame_rot)

def process_visual_element(lines, node, current_pos, current_rot, link_origin, link_rot):
    """
    Recursively process visual elements (Transform, Shape) and add to URDF.

    current_pos/current_rot: Current accumulated transform in global space
    link_origin/link_rot: The link's frame in global space (for computing relative transform)
    """
    if not isinstance(node, Node):
        return

    if node.type == 'Transform':
        t_trans = get_vec3(node, 'translation')
        t_ax, t_ang = get_rot(node, 'rotation')
        t_rot = axis_angle_to_matrix(t_ax, t_ang)

        new_pos = vec_add(current_pos, matrix_mul(current_rot, t_trans))
        new_rot = matrix_mult_mat(current_rot, t_rot)

        if 'children' in node.fields:
            for c in node.fields.get('children', []):
                process_visual_element(lines, c, new_pos, new_rot, link_origin, link_rot)

    elif node.type == 'Shape':
        add_visual_to_urdf(lines, node, current_pos, current_rot, link_origin, link_rot)

    elif node.type == 'Solid':
        pass

    elif node.type == 'HingeJoint':
        pass

def add_visual_to_urdf(lines, shape_node, global_pos, global_rot, link_origin, link_rot):
    """Add a visual/collision element to URDF from a Shape node."""
    geo = None
    if 'geometry' in shape_node.fields:
        g = shape_node.fields['geometry']
        if isinstance(g, Node) and g.type == 'Mesh':
            url = g.fields.get('url', '').replace('"', '')
            if '/pibsim_webots/' in url:
                url = os.path.join(os.getcwd(), 'pibsim_webots', url.split('/pibsim_webots/')[1])
            elif '/ros2_ws/pibsim_webots/' in url:
                url = os.path.join(os.getcwd(), 'pibsim_webots', url.split('/ros2_ws/pibsim_webots/')[1])
            elif '/app/ros2_ws/pibsim_webots/' in url:
                url = os.path.join(os.getcwd(), 'pibsim_webots', url.split('/app/ros2_ws/pibsim_webots/')[1])
            geo = f'<mesh filename="{url}"/>'

    if geo:
        link_rot_inv = matrix_transpose(link_rot)
        rel_pos = matrix_mul(link_rot_inv, vec_sub(global_pos, link_origin))
        rel_rot = matrix_mult_mat(link_rot_inv, global_rot)
        rpy = matrix_to_rpy(rel_rot)

        lines.append(f'    <visual>\n')
        lines.append(f'      <origin xyz="{rel_pos[0]} {rel_pos[1]} {rel_pos[2]}" rpy="{rpy[0]} {rpy[1]} {rpy[2]}"/>\n')
        lines.append(f'      <geometry>{geo}</geometry>\n')
        lines.append(f'    </visual>\n')
        lines.append(f'    <collision>\n')
        lines.append(f'      <origin xyz="{rel_pos[0]} {rel_pos[1]} {rel_pos[2]}" rpy="{rpy[0]} {rpy[1]} {rpy[2]}"/>\n')
        lines.append(f'      <geometry>{geo}</geometry>\n')
        lines.append(f'    </collision>\n')

def process_hinge_joint(lines, joint_node, parent_link_name, parent_link_origin, parent_link_rot, WEBOTS_URDF_OFFSET=-1.0):
    """
    Process a HingeJoint and its endpoint Solid.

    In Webots:
    - HingeJoint has anchor and axis in the PARENT solid's frame
    - endPoint Solid has translation/rotation relative to anchor point (at q=0)

    In URDF:
    - Joint origin is transform from parent link frame to joint frame
    - Axis is defined in the joint frame (which equals child link frame at q=0)
    """
    jp = joint_node.fields.get('jointParameters')
    if not isinstance(jp, Node):
        return

    anchor_local = get_vec3(jp, 'anchor')
    axis_local = get_vec3(jp, 'axis', [0, 0, 1])

    jname = "joint"
    min_position = 0.0
    max_position = 1.5708
    max_velocity = 20.0
    max_effort = 10.0

    if 'device' in joint_node.fields:
        devices = joint_node.fields['device']
        if isinstance(devices, list):
            for d in devices:
                if isinstance(d, Node) and d.type == 'RotationalMotor':
                    jname = d.fields.get('name', '"joint"').replace('"', '')
                    if 'minPosition' in d.fields:
                        min_position = float(d.fields['minPosition'])
                    if 'maxPosition' in d.fields:
                        max_position = float(d.fields['maxPosition'])
                    if 'maxVelocity' in d.fields:
                        max_velocity = float(d.fields['maxVelocity'])
                    if 'maxTorque' in d.fields:
                        max_effort = float(d.fields['maxTorque'])
                    break

    ep = joint_node.fields.get('endPoint')
    if not isinstance(ep, Node):
        return

    child_link_name = ep.fields.get('name', f'"{jname}_link"').replace('"', '')

    ep_trans = get_vec3(ep, 'translation')
    ep_rot_axis, ep_rot_angle = get_rot(ep, 'rotation')
    ep_rot = axis_angle_to_matrix(ep_rot_axis, ep_rot_angle)

    anchor_global = vec_add(parent_link_origin, matrix_mul(parent_link_rot, anchor_local))
    axis_global = normalize(matrix_mul(parent_link_rot, axis_local))

    child_global_pos = vec_add(parent_link_origin, matrix_mul(parent_link_rot, ep_trans))
    child_global_rot = matrix_mult_mat(parent_link_rot, ep_rot)

    joint_pos = anchor_local

    parent_rot_inv = matrix_transpose(parent_link_rot)
    relative_rot = matrix_mult_mat(parent_rot_inv, child_global_rot)

    axis_in_relative = normalize(matrix_mul(matrix_transpose(relative_rot), axis_local))
    offset_rot = axis_angle_to_matrix(axis_in_relative, WEBOTS_URDF_OFFSET)

    adjusted_relative_rot = matrix_mult_mat(relative_rot, offset_rot)
    joint_rpy = matrix_to_rpy(adjusted_relative_rot)

    adjusted_child_rot = matrix_mult_mat(parent_link_rot, adjusted_relative_rot)
    adjusted_child_rot_inv = matrix_transpose(adjusted_child_rot)
    axis_in_child = normalize(matrix_mul(adjusted_child_rot_inv, matrix_mul(parent_link_rot, axis_local)))

    lines.append(f'  <joint name="{jname}" type="revolute">\n')
    lines.append(f'    <parent link="{parent_link_name}"/>\n')
    lines.append(f'    <child link="{child_link_name}"/>\n')
    lines.append(f'    <origin xyz="{joint_pos[0]} {joint_pos[1]} {joint_pos[2]}" rpy="{joint_rpy[0]} {joint_rpy[1]} {joint_rpy[2]}"/>\n')
    lines.append(f'    <axis xyz="{axis_in_child[0]} {axis_in_child[1]} {axis_in_child[2]}"/>\n')
    lines.append(f'    <limit lower="{min_position}" upper="{max_position}" effort="{max_effort}" velocity="{max_velocity}"/>\n')
    lines.append(f'  </joint>\n')

    process_solid_as_link(
        lines,
        solid_node=ep,
        link_name=child_link_name,
        parent_frame_origin=parent_link_origin,
        parent_frame_rot=parent_link_rot
    )

def convert_proto_to_urdf(
    proto_path: str = "pibsim_webots/protos/pib.proto",
    urdf_path: str = "pib_model.urdf",
) -> str:
    """
    Convert a Webots .proto file to URDF format.

    This is a preprocessing tool for regenerating the URDF from the
    original Webots robot definition file.

    Args:
        proto_path: Path to the Webots .proto file.
        urdf_path: Path to write the output URDF file.

    Returns:
        The generated URDF as a string.

    Raises:
        FileNotFoundError: If proto_path doesn't exist.

    Example:
        >>> from pib3.tools import convert_proto_to_urdf
        >>> urdf = convert_proto_to_urdf(
        ...     "pibsim_webots/protos/pib.proto",
        ...     "pib_model.urdf"
        ... )
    """
    if not os.path.exists(proto_path):
        raise FileNotFoundError(f"Proto file not found: {proto_path}")

    print(f"Reading proto file: {proto_path}")
    with open(proto_path) as f:
        text = f.read()

    print("Parsing proto file...")
    robot_node = parse_proto(tokenize(text))

    print("Generating URDF...")
    urdf_str = generate_urdf(robot_node)

    with open(urdf_path, "w") as f:
        f.write(urdf_str)

    print(f"URDF generated: {urdf_path}")

    joint_count = urdf_str.count('<joint name=')
    link_count = urdf_str.count('<link name=')
    print(f"Generated {link_count} links and {joint_count} joints")

    return urdf_str


def main():
    """Main function to convert proto to URDF."""
    convert_proto_to_urdf()


if __name__ == '__main__':
    main()
