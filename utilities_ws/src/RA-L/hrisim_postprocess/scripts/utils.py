from enum import Enum


class TOD(Enum):
    STARTING = "starting"
    MORNING = "morning"
    LUNCH = "lunch"
    AFTERNOON = "afternoon"
    QUITTING = "quitting"
    OFF = "off"


class WP(Enum):
  PARKING = "parking"
  DOOR_ENTRANCE = "door_entrance"
  DOOR_ENTRANCE_CANTEEN = "door_entrance-canteen"
  CORRIDOR_ENTRANCE = "corridor_entrance"
  DOOR_CORRIDOR1 = "door_corridor1"
  DOOR_CORRIDOR2 = "door_corridor2"
  DOOR_CORRIDOR3 = "door_corridor3"
  SHELF12 = "shelf12"
  SHELF23 = "shelf23"
  SHELF34 = "shelf34"
  SHELF45 = "shelf45"
  SHELF56 = "shelf56"
  DOOR_OFFICE1 = "door_office1"
  DOOR_OFFICE2 = "door_office2"
  DOOR_TOILET1 = "door_toilet1"
  DOOR_TOILET2 = "door_toilet2"
  DELIVERY_POINT = "delivery_point"
  CORRIDOR0 = "corridor0"
  CORRIDOR1 = "corridor1"
  CORRIDOR2 = "corridor2"
  CORRIDOR3 = "corridor3"
  CORRIDOR4 = "corridor4"
  CORRIDOR5 = "corridor5"
  ENTRANCE = "entrance"
  OFFICE1 = "office1"
  OFFICE2 = "office2"
  TOILET1 = "toilet1"
  TOILET2 = "toilet2"
  TABLE2 = "table2"
  TABLE3 = "table3"
  TABLE4 = "table4"
  TABLE5 = "table5"
  TABLE6 = "table6"
  CORR_CANTEEN_1 = "corr_canteen_1"
  CORR_CANTEEN_2 = "corr_canteen_2"
  CORR_CANTEEN_3 = "corr_canteen_3"
  CORR_CANTEEN_4 = "corr_canteen_4"
  CORR_CANTEEN_5 = "corr_canteen_5"
  CORR_CANTEEN_6 = "corr_canteen_6"
  KITCHEN_1 = "kitchen1"
  KITCHEN_2 = "kitchen2"
  KITCHEN_3 = "kitchen3"
  CORRIDOR_CANTEEN = "corridor_canteen"
  SHELF1 = "shelf1"
  SHELF2 = "shelf2"
  SHELF3 = "shelf3"
  SHELF4 = "shelf4"
  SHELF5 = "shelf5"
  SHELF6 = "shelf6"
  CHARGING_STATION = "charging_station"
  
  
class NODES(Enum):
  TOD = 0
  R_V = 1
  R_B = 2
  # NP = 3
  PD = 3
  BAC = 4
  WP = 5
# class NODES(Enum):
#   TOD = 0
#   R_V = 1
#   R_T = 2
#   R_B = 3
#   NP = 4
#   PD = 5
#   BAC = 6
#   WP = 7