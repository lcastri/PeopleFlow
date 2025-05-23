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
  
class TaskResult(Enum):
    SUCCESS = 1
    FAILURE = -1
    WIP = 0

class Task(Enum):
    DELIVERY = "delivery"
    INVENTORY = "inventory"
    CLEANING = "cleaning"
    CHARGING = "charging"
    
    
TASKS = {
    Task.DELIVERY.value: 0,
    Task.INVENTORY.value: 1,
    Task.CLEANING.value: 2,
    Task.CHARGING.value: 3
}

TODS = {
    TOD.STARTING.value: 0,
    TOD.MORNING.value: 1,
    TOD.LUNCH.value: 2,
    TOD.AFTERNOON.value: 3,
    TOD.QUITTING.value: 4,
    TOD.OFF.value: 5
}

WPS = {
    WP.PARKING.value: 0,
    WP.DOOR_ENTRANCE.value: 1,
    WP.DOOR_ENTRANCE_CANTEEN.value: 2,
    WP.CORRIDOR_ENTRANCE.value: 3,
    WP.DOOR_CORRIDOR1.value: 4,
    WP.DOOR_CORRIDOR2.value: 5,
    WP.DOOR_CORRIDOR3.value: 6,
    WP.SHELF12.value: 7,
    WP.SHELF23.value: 8,
    WP.SHELF34.value: 9,
    WP.SHELF45.value: 10,
    WP.SHELF56.value: 11,
    WP.DOOR_OFFICE1.value: 12,
    WP.DOOR_OFFICE2.value: 13,
    WP.DOOR_TOILET1.value: 14,
    WP.DOOR_TOILET2.value: 15,
    WP.DELIVERY_POINT.value: 16,
    WP.CORRIDOR0.value: 17,
    WP.CORRIDOR1.value: 18,
    WP.CORRIDOR2.value: 19,
    WP.CORRIDOR3.value: 20,
    WP.CORRIDOR4.value: 21,
    WP.CORRIDOR5.value: 22,
    WP.ENTRANCE.value: 23,
    WP.OFFICE1.value: 24,
    WP.OFFICE2.value: 25,
    WP.TOILET1.value: 26,
    WP.TOILET2.value: 27,
    WP.TABLE2.value: 28,
    WP.TABLE3.value: 29,
    WP.TABLE4.value: 30,
    WP.TABLE5.value: 31,
    WP.TABLE6.value: 32,
    WP.CORR_CANTEEN_1.value: 33,
    WP.CORR_CANTEEN_2.value: 34,
    WP.CORR_CANTEEN_3.value: 35,
    WP.CORR_CANTEEN_4.value: 36,
    WP.CORR_CANTEEN_5.value: 37,
    WP.CORR_CANTEEN_6.value: 38,
    WP.KITCHEN_1.value: 39,
    WP.KITCHEN_2.value: 40,
    WP.KITCHEN_3.value: 41,
    WP.CORRIDOR_CANTEEN.value: 42,
    WP.SHELF1.value: 43,
    WP.SHELF2.value: 44,
    WP.SHELF3.value: 45,
    WP.SHELF4.value: 46,
    WP.SHELF5.value: 47,
    WP.SHELF6.value: 48,
    WP.CHARGING_STATION.value: 49,
}