#!/usr/bin/env python

import math
import util

_WALL_SECTION_OWL_TEMPLATE = """
{} rdf:type owl:NamedIndividual ,
        paintbot:WallSection ;
    skiros:OrientationX "{}"^^xsd:float ;
    skiros:OrientationY "{}"^^xsd:float ;
    skiros:OrientationZ "{}"^^xsd:float ;
    skiros:OrientationW "{}"^^xsd:float ;
    skiros:PositionX "{}"^^xsd:float ;
    skiros:PositionY "{}"^^xsd:float ;
    rdfs:label "Wall_{}"^^xsd:string ;
    paintbot:targetColor {} .
"""

def gen_wall_centers(p1, p2, width):
    # Normalized wall vector
    w = (p2[0] - p1[0], p2[1] - p1[1])
    w_mag = math.sqrt(w[0]**2 + w[1]**2)
    w = (w[0] / w_mag, w[1] / w_mag)
    # Calculate section centers
    num_secs = math.ceil(w_mag / width)
    sec_width = w_mag / num_secs
    centers = []
    for n in range(1, num_secs + 1):
        d = (n * sec_width) - (sec_width / 2)
        # Clamp d
        if d - (width / 2) < 0:
            d += (width / 2) - (sec_width / 2)
        elif d + (width / 2) > w_mag:
            d -= (width / 2) - (sec_width / 2)
        centers.append(((w[0] * d) + p1[0], (w[1] * d) + p1[1]))
    return centers

def generate(p1, p2, width, target_color, id_start):
    # Note: Orientation of sections is theta(p2-p1) - math.pi/2
    centers = gen_wall_centers(p1, p2, width)
    id = id_start
    owl = []
    for c in centers:
        full_id = 'paintbot:WallSection-{}'.format(id)
        q = util.euler_to_quaternion(0, 0, math.atan2(p2[1] - p1[1], p2[0] - p1[0]) - (math.pi / 2))
        owl.append((
            full_id,
            _WALL_SECTION_OWL_TEMPLATE.format(
                full_id,
                q[0], q[1], q[2], q[3],
                c[0], c[1],
                id,
                target_color)))
        id += 1
    return owl
