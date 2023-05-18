#!/usr/bin/env python

#@prefix paintbot: <http://unomaha.edu/ontologies/paintbot#> .
#@prefix owl: <http://www.w3.org/2002/07/owl#> .
#@prefix rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> .
#@prefix xml: <http://www.w3.org/XML/1998/namespace> .
#@prefix xsd: <http://www.w3.org/2001/XMLSchema#> .
#@prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#> .
#@prefix skiros: <http://rvmi.aau.dk/ontologies/skiros.owl#> .

import paint
import wall_section

_PAINT_ROLLER_WIDTH = 0.2286
_PAINT_ID_START = 1
_TRAY_ID_START = 51
_WALL_SECTION_ID_START = 100
_SCENE_OWL_TEMPLATE = """
@prefix husky: <http://www.semanticweb.org/zeus/ontologies/husky#> .
@prefix owl: <http://www.w3.org/2002/07/owl#> .
@prefix rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> .
@prefix xml: <http://www.w3.org/XML/1998/namespace> .
@prefix xsd: <http://www.w3.org/2001/XMLSchema#> .
@prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#> .
@prefix skiros: <http://rvmi.aau.dk/ontologies/skiros.owl#> .

# Scene
skiros:Scene-0 rdf:type owl:NamedIndividual ,
        skiros:Scene ;
{}
{}
    skiros:DiscreteReasoner "AauSpatialReasoner"^^xsd:string ;
    skiros:FrameId "map"^^xsd:string ;
    skiros:OrientationW "1.0"^^xsd:float ;
    skiros:OrientationX "0.0"^^xsd:float ;
    skiros:OrientationY "0.0"^^xsd:float ;
    skiros:OrientationZ "0.0"^^xsd:float ;
    skiros:PositionX "0.0"^^xsd:float ;
    skiros:PositionY "0.0"^^xsd:float ;
    skiros:PositionZ "0.0"^^xsd:float ;
    rdfs:label "Scene"^^xsd:string .

# Paints
{}

# Trays
{}

# Wall sections
{}
"""

def generate(paints, segments):
    # Paints and trays
    paints_owl = ''
    trays_owl = ''
    tray_contains = []
    paint_id = _PAINT_ID_START
    tray_id = _TRAY_ID_START
    paint_ids = {}
    for p in paints:
        pnt, tray = paint.generate(p[0], p[1], p[2], paint_id, tray_id)
        paint_id += 1
        tray_id += 1
        paint_ids[p[0]] = pnt[0]
        paints_owl += pnt[1]
        trays_owl += tray[1]
        tray_contains.append('    skiros:contain {} ;'.format(tray[0]))

    # Wall sections
    wall_sections = []
    id_start = _WALL_SECTION_ID_START
    for seg in segments:
        sections = wall_section.generate(
            seg[0], seg[1],
            _PAINT_ROLLER_WIDTH,
            paint_ids[seg[2]],
            id_start)
        id_start += len(sections)
        wall_sections += sections
    ws_contains = ['    skiros:contain {} ;'.format(ws[0]) for ws in wall_sections]

    ws_owl = ''
    for ws in wall_sections:
        ws_owl += ws[1]

    # Construct full scene
    return _SCENE_OWL_TEMPLATE.format(
        '\n'.join(tray_contains),
        '\n'.join(ws_contains),
        paints_owl,
        trays_owl,
        ws_owl)
