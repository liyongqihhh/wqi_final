"""Add real deck contact only to the generated PX4 cooperative UGV model."""

from copy import deepcopy
import xml.etree.ElementTree as ET


def add_physical_deck(robot_xml: str) -> str:
    root = ET.fromstring(robot_xml)
    base = root.find("link[@name='base_link']")
    joint = root.find("joint[@name='base_joint']/origin")
    if base is None or joint is None:
        raise ValueError("Carrier model requires base_link and base_joint")
    deck = base.find("visual[@name='uav_docking_deck_visual']")
    if deck is None:
        raise ValueError("Carrier visual deck is unavailable")
    origin = deck.find("origin")
    box = deck.find("geometry/box")
    if origin is None or box is None:
        raise ValueError("Carrier deck must be a box with an origin")
    # Iris's flat body collision has a 0.055 m lower extent. Keep the existing
    # 0.42 m docking base height, with no initial deck/body interpenetration.
    base_height = float(joint.get("xyz").split()[2])
    deck_z = 0.42 - base_height - 0.055 - 0.02
    origin.set("xyz", f"0 0 {deck_z:.9f}")
    # Full diagonal remains below the unchanged 0.60 m navigation radius.
    box.set("size", "0.75 0.75 0.04")
    if base.find("collision[@name='px4_carrier_deck_collision']") is None:
        collision = ET.SubElement(base, "collision", {
            "name": "px4_carrier_deck_collision"})
        collision.append(deepcopy(origin))
        collision.append(deepcopy(deck.find("geometry")))
    return ET.tostring(root, encoding="unicode")
