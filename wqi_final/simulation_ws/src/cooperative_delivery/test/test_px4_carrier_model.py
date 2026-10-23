import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro

from cooperative_delivery.px4_carrier_model import add_physical_deck


def test_generated_deck_supports_iris_inside_navigation_envelope():
    source = Path(__file__).parents[2] / "ugvcar_description/urdf/ugvcar/ugvcar.urdf.xacro"
    original = xacro.process_file(str(source)).toxml()
    model = ET.fromstring(add_physical_deck(original))
    collision = model.find("link[@name='base_link']/collision[@name='px4_carrier_deck_collision']")
    sx, sy, thickness = map(float, collision.find("geometry/box").get("size").split())
    assert math.hypot(sx / 2, sy / 2) < .60
    z = float(collision.find("origin").get("xyz").split()[2])
    base_z = float(model.find("joint[@name='base_joint']/origin").get("xyz").split()[2])
    assert z + base_z + thickness / 2 == pytest.approx(.42 - .055)
    assert ET.fromstring(original).find("link[@name='base_link']/collision[@name='px4_carrier_deck_collision']") is None
    repeated = ET.fromstring(add_physical_deck(ET.tostring(model, encoding="unicode")))
    assert len(repeated.findall("link[@name='base_link']/collision[@name='px4_carrier_deck_collision']")) == 1
    assert model.find("link[@name='base_link']/inertial/mass").get("value") == "1.0"


def test_missing_carrier_geometry_is_not_silently_accepted():
    with pytest.raises(ValueError):
        add_physical_deck('<robot name="empty"/>')
