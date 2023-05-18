#!/usr/bin/env python

import util

_PAINT_OWL_TEMPLATE = """
{} rdf:type owl:NamedIndividual ,
        paintbot:Paint ;
    rdfs:label "Paint_{}"^^xsd:string .
"""
_TRAY_OWL_TEMPLATE = """
{} rdf:type owl:NamedIndividual ,
        paintbot:PaintTray ;
    skiros:contain {} ;
    skiros:OrientationX "{}"^^xsd:float ;
    skiros:OrientationY "{}"^^xsd:float ;
    skiros:OrientationZ "{}"^^xsd:float ;
    skiros:OrientationW "{}"^^xsd:float ;
    skiros:PositionX "{}"^^xsd:float ;
    skiros:PositionY "{}"^^xsd:float ;
    rdfs:label "PaintTray_{}"^^xsd:string .
"""

def generate(paint, tray_loc, tray_ang, paint_id, tray_id):
    paint_full_id = 'paintbot:Paint-{}'.format(paint_id)
    paint_owl = _PAINT_OWL_TEMPLATE.format(paint_full_id, paint)

    q = util.euler_to_quaternion(0, 0, tray_ang)
    tray_full_id = 'paintbot:PaintTray-{}'.format(tray_id)
    tray_owl = _TRAY_OWL_TEMPLATE.format(
        tray_full_id,
        paint_full_id,
        q[0], q[1], q[2], q[3],
        tray_loc[0], tray_loc[1],
        paint)

    return (paint_full_id, paint_owl), (tray_full_id, tray_owl)
