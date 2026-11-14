"""Regional traffic tied to actual campus coordinates, not legacy waypoints."""
import math


DENSITIES={"none":{"ground":0,"air":0},"low":{"ground":6,"air":3},
    "medium":{"ground":9,"air":6},"high":{"ground":12,"air":9},
    "very_high":{"ground":15,"air":12},"extreme":{"ground":18,"air":15}}


def traffic_config(layout):
    ground={
        "A":[((-39.5,19.),(2.1,2.5)),((-28.5,22.),(0.,3.)),
             ((-51.,22.),(6.,0.)),((-43.,34.),(3.2,0.)),
             ((-43.,48.),(0.,3.)),((-53.,48.),(6.,0.))],
        "B":[((38.,18.),(0.,3.)),((60.,26.),(3.,0.)),
             ((60.,40.),(7.,0.)),((54.,40.),(0.,3.)),
             ((66.,40.),(0.,3.)),((60.,34.),(0.,4.))],
        "C":[((12.,-38.),(3.,0.)),((0.,-38.),(0.,3.)),
             ((24.,-38.),(6.,0.)),((58.,-48.),(2.5,0.)),
             ((58.,-62.),(2.5,0.)),((46.,-66.),(5.,0.))]}
    routes={}
    for region, spec in layout.regions.items():
        for index,(point,normal) in enumerate(ground[region]):
            x,y=point;dx,dy=normal
            speed=.65 if index in (2,5) else .42+.03*(index%3)
            waypoints=[[x+dx,y+dy,.8],[x-dx,y-dy,.8]]
            if index==2:
                waypoints.reverse()
            # A longitudinal actor stages on the road itself. Release before
            # the robot reaches its parked endpoint, not only the midpoint.
            # Otherwise local collision protection can stop outside the gate.
            activation=([*waypoints[0][:2],.3] if index in (2,5) else [x,y,.3])
            activation_distance=(8. if index in (2,5)
                else .4*(math.hypot(dx,dy)/speed+speed/(2*1.2))+.3)
            routes[f"regional_{region}_ground_{index+1}"]={"layer":"ground","group":region,
                "priority":0 if index<2 else 1,"speed_mps":speed,
                "radius_m":.35,"height_m":1.6,
                "waypoints":waypoints,
                "activation_models":[layout.hub["namespace"],spec["namespace"]],
                "activation_point":activation,
                "activation_distance":activation_distance}
        source=layout.hub["uav_pad"][:2];target=spec["depot_pose"][:2]
        length=math.dist(source,target)
        nx,ny=-(target[1]-source[1])/length,(target[0]-source[0])/length
        height=layout.data["flight"]["cruise_altitude"]
        for index,fraction in enumerate((.45,.25,.62,.76,.88)):
            x=source[0]+fraction*(target[0]-source[0]);y=source[1]+fraction*(target[1]-source[1])
            # Oblique 3-D crossings really intersect the 15 m cruise plane.
            # Actors stage away from the corridor and move continuously after
            # a one-time proximity trigger. No teleport or robot yielding.
            span=5.0;dz=1.2 if index%2 else 0.
            routes[f"regional_{region}_air_{index+1}"]={"layer":"air","group":region,
                "priority":0 if index==0 else 1,"speed_mps":.75+.04*(index%3),
                "radius_m":.75,"height_m":1.5,
                "waypoints":[[x+nx*span,y+ny*span,height+dz],
                             [x-nx*span,y-ny*span,height-dz]],
                "activation_models":["campus_uav"],"activation_point":[x,y,height],
                "activation_distance":9.0}
    return {"densities":DENSITIES,"routes":routes,
            "protected_stops":{name:pose[:2] for name,pose in layout.all_stops().items()}}
