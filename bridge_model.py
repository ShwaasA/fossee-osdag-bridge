import math
import argparse

# Provided i_section file
from draw_i_section import create_i_section

# OpenCASCADE (pythonOCC) 
from OCC.Core.BRep import BRep_Builder
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakePrism
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform, BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire, BRepBuilderAPI_MakeFace
from OCC.Core.gp import gp_Vec, gp_Trsf, gp_Pnt, gp_Ax1, gp_Dir
from OCC.Core.TopoDS import TopoDS_Compound

from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Core.Interface import Interface_Static
from OCC.Display.SimpleGui import init_display

# COMPONENT FACTORIES
def create_rectangular_prism(width, height, length):
    box_maker = BRepPrimAPI_MakeBox(length, width, height)
    return box_maker.Shape()

def create_circular_pier(diameter, height):
    radius = diameter / 2.0
    cylinder_maker = BRepPrimAPI_MakeCylinder(radius, height)
    return cylinder_maker.Shape()

def create_trapezoidal_pier_cap(length, top_width, bottom_width, depth):
    p1 = gp_Pnt(0, -bottom_width / 2.0, 0)
    p2 = gp_Pnt(0, bottom_width / 2.0, 0)
    p3 = gp_Pnt(0, top_width / 2.0, depth)
    p4 = gp_Pnt(0, -top_width / 2.0, depth)

    e1 = BRepBuilderAPI_MakeEdge(p1, p2).Edge()
    e2 = BRepBuilderAPI_MakeEdge(p2, p3).Edge()
    e3 = BRepBuilderAPI_MakeEdge(p3, p4).Edge()
    e4 = BRepBuilderAPI_MakeEdge(p4, p1).Edge()

    wire = BRepBuilderAPI_MakeWire(e1, e2, e3, e4).Wire()
    face = BRepBuilderAPI_MakeFace(wire).Face()
    
    extrusion_vector = gp_Vec(length, 0, 0)
    prism_maker = BRepPrimAPI_MakePrism(face, extrusion_vector)
    return prism_maker.Shape()


# ASSEMBLY COMPONENTS
def move_shape(shape, move_x, move_y, move_z):
    translation_vector = gp_Vec(move_x, move_y, move_z)
    transformation = gp_Trsf()
    transformation.SetTranslation(translation_vector)
    transformer = BRepBuilderAPI_Transform(shape, transformation, True)
    return transformer.Shape()

def build_foundation(cap_length, cap_width, cap_depth, pile_diameter, pile_length, pile_spacing):
    compound_shape = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(compound_shape)

    # Pile Cap (Centered on X and Y)
    raw_cap = create_rectangular_prism(width=cap_width, height=cap_depth, length=cap_length)
    positioned_cap = move_shape(raw_cap, move_x=-(cap_length / 2.0), move_y=-(cap_width / 2.0), move_z=-cap_depth)
    builder.Add(compound_shape, positioned_cap)

    # 2x2 Piles (Symmetrical around center)
    offset = pile_spacing / 2.0
    x_positions = [-offset, offset]
    y_positions = [-offset, offset]
    pile_z_start = -cap_depth - pile_length

    for x in x_positions:
        for y in y_positions:
            raw_pile = create_circular_pier(diameter=pile_diameter, height=pile_length)
            positioned_pile = move_shape(raw_pile, move_x=x, move_y=y, move_z=pile_z_start)
            builder.Add(compound_shape, positioned_pile)

    return compound_shape

def build_substructure(pier_diameter, pier_height, cap_length, cap_top_width, cap_bottom_width, cap_depth,
                       pile_cap_length, pile_cap_width, pile_cap_depth, pile_diameter, pile_length, pile_spacing):
    """
    Groups the Foundation, Column, and Pier Cap into a single, perfectly centered solid compound.
    """
    compound_shape = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(compound_shape)

    # 1. Add the Foundation (Starts underground, ends at Z=0)
    foundation = build_foundation(pile_cap_length, pile_cap_width, pile_cap_depth, pile_diameter, pile_length, pile_spacing)
    builder.Add(compound_shape, foundation)

    # 2. Add the Pier (Starts at Z=0, goes to Z=pier_height)
    pier = create_circular_pier(diameter=pier_diameter, height=pier_height)
    builder.Add(compound_shape, pier)

    # 3. Add the Pier Cap (Sits on the pier, centered on X)
    raw_cap = create_trapezoidal_pier_cap(cap_length, cap_top_width, cap_bottom_width, cap_depth)
    # The cap is drawn stretching +X, so we shift it backwards by half its length to center it perfectly over the pier
    positioned_cap = move_shape(raw_cap, move_x=-(cap_length / 2.0), move_y=0, move_z=pier_height)
    builder.Add(compound_shape, positioned_cap)

    return compound_shape

def build_girders(n_girders, spacing, length, d, bf, tf, tw, deck_bottom_z):
    compound_shape = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(compound_shape)
    if spacing == 0: spacing = args.deck_width / args.n_girders

    total_width_of_girders = (n_girders - 1) * spacing
    start_y = -((total_width_of_girders + tw) / 2.0)

    for i in range(n_girders):
        raw_girder = create_i_section(length=length, width=bf, depth=d, flange_thickness=tf, web_thickness=tw)
        move_x = -(length / 2.0)
        current_y = start_y + (i * spacing)
        move_z = deck_bottom_z - d
        positioned_girder = move_shape(raw_girder, move_x, current_y, move_z)
        builder.Add(compound_shape, positioned_girder)

    return compound_shape

def create_rebar_grid_for_deck(deck_width, span_length, cover, main_diam, main_spacing, transverse_diam, transverse_spacing):
    compound_shape = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(compound_shape)

    grid_length_x = span_length - (2 * cover)
    grid_width_y = deck_width - (2 * cover)

    # Y Bars (Point along Y)
    y_axis_rotation = gp_Trsf()
    x_axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0))
    y_axis_rotation.SetRotation(x_axis, -math.pi / 2.0) 

    num_transverse = int(grid_length_x / transverse_spacing) + 1
    for i in range(num_transverse):
        current_x = cover + (i * transverse_spacing)
        raw_bar = create_circular_pier(diameter=transverse_diam, height=grid_width_y)
        rotated_bar = BRepBuilderAPI_Transform(raw_bar, y_axis_rotation, True).Shape()
        positioned_bar = move_shape(rotated_bar, move_x=current_x, move_y=cover, move_z=cover + (transverse_diam/2.0))
        builder.Add(compound_shape, positioned_bar)

    # X Bars (Point along X)
    x_axis_rotation = gp_Trsf()
    y_axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0))
    x_axis_rotation.SetRotation(y_axis, math.pi / 2.0)

    num_longitudinal = int(grid_width_y / main_spacing) + 1
    for i in range(num_longitudinal):
        current_y = cover + (i * main_spacing)
        raw_bar = create_circular_pier(diameter=main_diam, height=grid_length_x)
        rotated_bar = BRepBuilderAPI_Transform(raw_bar, x_axis_rotation, True).Shape()
        z_height = cover + transverse_diam + (main_diam/2.0)
        positioned_bar = move_shape(rotated_bar, move_x=cover, move_y=current_y, move_z=z_height)
        builder.Add(compound_shape, positioned_bar)

    return compound_shape

def build_parapets(deck_width, deck_thickness, span_length, parapet_width=300, parapet_height=800):
    compound_shape = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(compound_shape)

    raw_parapet = create_rectangular_prism(width=parapet_width, height=parapet_height, length=span_length)

    pos_left = move_shape(raw_parapet, -(span_length/2.0), -(deck_width/2.0), deck_thickness)
    builder.Add(compound_shape, pos_left)

    pos_right = move_shape(raw_parapet, -(span_length/2.0), (deck_width/2.0) - parapet_width, deck_thickness)
    builder.Add(compound_shape, pos_right)

    return compound_shape

# CLI PARSING
def parse_arguments():
    """Parses command line arguments to make the bridge model fully parametric from the terminal."""
    parser = argparse.ArgumentParser(description="Parametric 3D CAD Model of a Steel Girder Bridge")
    
    # Deck
    parser.add_argument("--span_length", type=float, default=12000, help="Total clear span length (mm)")
    parser.add_argument("--deck_width", type=float, default=7000, help="Total deck width (mm)")
    parser.add_argument("--deck_thickness", type=float, default=250, help="Thickness of the deck slab (mm)")
    
    # Pier & Cap
    parser.add_argument("--pier_diameter", type=float, default=800, help="Diameter of the circular pier (mm)")
    parser.add_argument("--pier_height", type=float, default=3000, help="Height of the pier from pile cap to pier cap (mm)")
    parser.add_argument("--pier_cap_length", type=float, default=1000, help="Length (thickness) of the pier cap (mm)")
    parser.add_argument("--pier_cap_top_width", type=float, default=6900, help="Top width of the pier cap (mm)")
    parser.add_argument("--pier_cap_bottom_width", type=float, default=6000, help="Bottom width of the pier cap (mm)")
    parser.add_argument("--pier_cap_depth", type=float, default=600, help="Depth (height) of the pier cap (mm)")
    
    # Girders
    parser.add_argument("--n_girders", type=int, default=3, help="Number of main longitudinal girders")
    parser.add_argument("--girder_spacing", type=float, default=0, help="Center-to-center spacing of main girders (mm)")
    parser.add_argument("--girder_d", type=float, default=900, help="Depth of main girder I-section (mm)")
    parser.add_argument("--girder_bf", type=float, default=300, help="Flange width of main girder (mm)")
    parser.add_argument("--girder_tf", type=float, default=16, help="Flange thickness of main girder (mm)")
    parser.add_argument("--girder_tw", type=float, default=200, help="Web thickness of main girder (mm)")
    
    # Foundation
    parser.add_argument("--pile_cap_length", type=float, default=2200, help="Length of the pile cap (mm)")
    parser.add_argument("--pile_cap_width", type=float, default=2200, help="Width of the pile cap (mm)")
    parser.add_argument("--pile_cap_depth", type=float, default=600, help="Depth of the pile cap (mm)")
    parser.add_argument("--pile_diameter", type=float, default=400, help="Diameter of individual piles (mm)")
    parser.add_argument("--pile_length", type=float, default=5000, help="Length (depth) of individual piles (mm)")
    parser.add_argument("--pile_spacing", type=float, default=1300, help="Spacing between pile centers (mm)")
    
    # Rebar
    parser.add_argument("--rebar_main_diameter", type=float, default=20, help="Diameter of longitudinal rebar (mm)")
    parser.add_argument("--rebar_spacing_longitudinal", type=float, default=150, help="Spacing of longitudinal rebar (mm)")
    parser.add_argument("--rebar_transverse_diameter", type=float, default=20, help="Diameter of transverse rebar (mm)")
    parser.add_argument("--rebar_spacing_transverse", type=float, default=200, help="Spacing of transverse rebar (mm)")
    parser.add_argument("--rebar_cover", type=float, default=40, help="Concrete clear cover for rebar (mm)")
    
    return parser.parse_args()


# MAIN EXECUTION & VISUALIZATION
if __name__ == "__main__":
    args = parse_arguments()
    display, start_display, add_menu, add_function_to_menu = init_display()
    display.set_bg_gradient_color([255, 255, 255], [255, 255, 255])
    
    # Substructure (Foundation)
    master_substructure = build_substructure(
        args.pier_diameter, args.pier_height, args.pier_cap_length, 
        args.pier_cap_top_width, args.pier_cap_bottom_width, args.pier_cap_depth,
        args.pile_cap_length, args.pile_cap_width, args.pile_cap_depth, 
        args.pile_diameter, args.pile_length, args.pile_spacing
    )

    cap_inset = args.pier_cap_length / 2.0 
    left_x_position = -(args.span_length / 2.0) + cap_inset
    right_x_position = (args.span_length / 2.0) - cap_inset

    substructure_left = move_shape(master_substructure, move_x=left_x_position, move_y=0, move_z=0)
    substructure_right = move_shape(master_substructure, move_x=right_x_position, move_y=0, move_z=0)

    # Girders
    cap_top_z = args.pier_height + args.pier_cap_depth
    girder_bottom_z = cap_top_z 
    
    all_girders = build_girders(
        args.n_girders, args.girder_spacing, args.span_length, args.girder_d, 
        args.girder_bf, args.girder_tf, args.girder_tw, girder_bottom_z + args.girder_d
    )

    # Deck
    deck_z_position = cap_top_z + args.girder_d
    raw_deck = create_rectangular_prism(width=args.deck_width, height=args.deck_thickness, length=args.span_length)
    positioned_deck = move_shape(raw_deck, -(args.span_length / 2.0), -(args.deck_width / 2.0), deck_z_position)

    parapets = build_parapets(args.deck_width, args.deck_thickness, args.span_length)
    positioned_parapets = move_shape(parapets, 0, 0, deck_z_position)

    # Rebar
    raw_rebar = create_rebar_grid_for_deck(
        args.deck_width, args.span_length, args.rebar_cover, args.rebar_main_diameter, 
        args.rebar_spacing_longitudinal, args.rebar_transverse_diameter, args.rebar_spacing_transverse
    )
    positioned_rebar = move_shape(raw_rebar, -(args.span_length / 2.0), -(args.deck_width / 2.0), deck_z_position)
        
    # Concrete Parts (semi-transparent)
    concrete_alpha = 0.4
    display.DisplayShape(positioned_deck, color="BLACK", transparency=concrete_alpha)
    display.DisplayShape(positioned_parapets, color="BLACK", transparency=concrete_alpha)
    display.DisplayShape(substructure_left, color="BLACK", transparency=concrete_alpha)
    display.DisplayShape(substructure_right, color="BLACK", transparency=concrete_alpha)
    
    # Steel Parts (opaque)
    display.DisplayShape(all_girders, color="BLACK")
    display.DisplayShape(positioned_rebar, color="BLACK")
    
    display.FitAll()
    start_display()