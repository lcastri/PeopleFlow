import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt

# Load the XML file
xml_data = """
<schedule>
    <!-- Time resolution: seconds -->
    <time name="H1" duration="300" >    <!-- 01h : 8am - 9am -->
      <adddest name="parking"         p="0.05" std="0" />
      <adddest name="office-1"        p="0.025" std="0" />
      <adddest name="office-2"        p="0.025" std="0" />
      <adddest name="toilet-1"        p="0.05" std="0" />
      <adddest name="toilet-2"        p="0.05" std="0" />
      <adddest name="wa-1-l"          p="0" std="0"/>
      <adddest name="wa-1-c"          p="0" std="0"/>
      <adddest name="wa-1-r"          p="0" std="0"/>
      <adddest name="wa-2-l"          p="0.35" std="0"/>
      <adddest name="wa-2-c"          p="0" std="0"/>
      <adddest name="wa-2-r"          p="0" std="0"/>
      <adddest name="wa-3-l"          p="0" std="0"/>
      <adddest name="wa-3-c"          p="0" std="0"/>
      <adddest name="wa-3-r"          p="0" std="0"/>
      <adddest name="wa-4-l"          p="0.35" std="0"/>
      <adddest name="wa-4-c"          p="0" std="0"/>
      <adddest name="wa-4-r"          p="0" std="0"/>
      <adddest name="wa-5-l"          p="0" std="0"/>
      <adddest name="wa-5-c"          p="0" std="0"/>
      <adddest name="wa-5-r"          p="0" std="0"/>
      <adddest name="target-1"        p="0" std="0"/>
      <adddest name="target-2"        p="0" std="0"/>
      <adddest name="target-3"        p="0" std="0"/>
      <adddest name="target-4"        p="0" std="0"/>
      <adddest name="target-5"        p="0" std="0"/>
      <adddest name="target-6"        p="0" std="0"/>
      <adddest name="target-7"        p="0" std="0"/>
      <adddest name="table-12"        p="0.025" std="0" />
      <adddest name="table-23"        p="0.025" std="0" />
      <adddest name="corridor-canteen-2"       p="0.025" std="0" />
      <adddest name="corridor-canteen-3"       p="0.025" std="0" />
    </time>
    <!-- 02h : 9am - 10am -->
    <time name="H2" duration="300" >
      <adddest name="parking"         p="0" std="0" />
      <adddest name="office-1"        p="0.05" std="0" />
      <adddest name="office-2"        p="0.05" std="0" />
      <adddest name="toilet-1"        p="0.05" std="0" />
      <adddest name="toilet-2"        p="0.05" std="0" />
      <adddest name="wa-1-l"          p="0" std="0"/>
      <adddest name="wa-1-c"          p="0" std="0"/>
      <adddest name="wa-1-r"          p="0" std="0"/>
      <adddest name="wa-2-l"          p="0" std="0"/>
      <adddest name="wa-2-c"          p="0.38" std="0"/>
      <adddest name="wa-2-r"          p="0" std="0"/>
      <adddest name="wa-3-l"          p="0" std="0"/>
      <adddest name="wa-3-c"          p="0" std="0"/>
      <adddest name="wa-3-r"          p="0" std="0"/>
      <adddest name="wa-4-l"          p="0" std="0"/>
      <adddest name="wa-4-c"          p="0.38" std="0"/>
      <adddest name="wa-4-r"          p="0" std="0"/>
      <adddest name="wa-5-l"          p="0" std="0"/>
      <adddest name="wa-5-c"          p="0" std="0"/>
      <adddest name="wa-5-r"          p="0" std="0"/>
      <adddest name="target-1"        p="0" std="0"/>
      <adddest name="target-2"        p="0" std="0"/>
      <adddest name="target-3"        p="0" std="0"/>
      <adddest name="target-4"        p="0" std="0"/>
      <adddest name="target-5"        p="0" std="0"/>
      <adddest name="target-6"        p="0" std="0"/>
      <adddest name="target-7"        p="0" std="0"/>
      <adddest name="table-12"        p="0.01" std="0" />
      <adddest name="table-23"        p="0.01" std="0" />
      <adddest name="corridor-canteen-2"       p="0.01" std="0" />
      <adddest name="corridor-canteen-3"       p="0.01" std="0" />
    </time>
    <!-- 03h : 10am - 11am -->
    <time name="H3" duration="300" >
      <adddest name="parking"         p="0" std="0" />
      <adddest name="office-1"        p="0.05" std="0" />
      <adddest name="office-2"        p="0.05" std="0" />
      <adddest name="toilet-1"        p="0.05" std="0.025" />
      <adddest name="toilet-2"        p="0.05" std="0.025" />
      <adddest name="wa-1-l"          p="0" std="0"/>
      <adddest name="wa-1-c"          p="0" std="0"/>
      <adddest name="wa-1-r"          p="0" std="0"/>
      <adddest name="wa-2-l"          p="0" std="0"/>
      <adddest name="wa-2-c"          p="0" std="0"/>
      <adddest name="wa-2-r"          p="0.38" std="0"/>
      <adddest name="wa-3-l"          p="0" std="0"/>
      <adddest name="wa-3-c"          p="0" std="0"/>
      <adddest name="wa-3-r"          p="0" std="0"/>
      <adddest name="wa-4-l"          p="0" std="0"/>
      <adddest name="wa-4-c"          p="0" std="0"/>
      <adddest name="wa-4-r"          p="0.38" std="0"/>
      <adddest name="wa-5-l"          p="0" std="0"/>
      <adddest name="wa-5-c"          p="0" std="0"/>
      <adddest name="wa-5-r"          p="0" std="0"/>
      <adddest name="target-1"        p="0" std="0"/>
      <adddest name="target-2"        p="0" std="0"/>
      <adddest name="target-3"        p="0" std="0"/>
      <adddest name="target-4"        p="0" std="0"/>
      <adddest name="target-5"        p="0" std="0"/>
      <adddest name="target-6"        p="0" std="0"/>
      <adddest name="target-7"        p="0" std="0"/>
      <adddest name="table-12"        p="0.01" std="0.005" />
      <adddest name="table-23"        p="0.01" std="0.005" />
      <adddest name="corridor-canteen-2"       p="0.01" std="0.005" />
      <adddest name="corridor-canteen-3"       p="0.01" std="0.005" />
    </time>
    <!-- 04h : 11am - 12am -->
    <time name="H4" duration="300" >
      <adddest name="parking"         p="0" std="0" />
      <adddest name="office-1"        p="0.05" std="0" />
      <adddest name="office-2"        p="0.05" std="0" />
      <adddest name="toilet-1"        p="0.05" std="0" />
      <adddest name="toilet-2"        p="0.05" std="0" />
      <adddest name="wa-1-l"          p="0" std="0"/>
      <adddest name="wa-1-c"          p="0" std="0"/>
      <adddest name="wa-1-r"          p="0" std="0"/>
      <adddest name="wa-2-l"          p="0" std="0"/>
      <adddest name="wa-2-c"          p="0" std="0"/>
      <adddest name="wa-2-r"          p="0" std="0"/>
      <adddest name="wa-3-l"          p="0.38" std="0"/>
      <adddest name="wa-3-c"          p="0" std="0"/>
      <adddest name="wa-3-r"          p="0" std="0"/>
      <adddest name="wa-4-l"          p="0" std="0"/>
      <adddest name="wa-4-c"          p="0.38" std="0"/>
      <adddest name="wa-4-r"          p="0" std="0"/>
      <adddest name="wa-5-l"          p="0" std="0"/>
      <adddest name="wa-5-c"          p="0" std="0"/>
      <adddest name="wa-5-r"          p="0" std="0"/>
      <adddest name="target-1"        p="0" std="0"/>
      <adddest name="target-2"        p="0" std="0"/>
      <adddest name="target-3"        p="0" std="0"/>
      <adddest name="target-4"        p="0" std="0"/>
      <adddest name="target-5"        p="0" std="0"/>
      <adddest name="target-6"        p="0" std="0"/>
      <adddest name="target-7"        p="0" std="0"/>
      <adddest name="table-12"        p="0.01" std="0" />
      <adddest name="table-23"        p="0.01" std="0" />
      <adddest name="corridor-canteen-2"       p="0.01" std="0" />
      <adddest name="corridor-canteen-3"       p="0.01" std="0" />
    </time>
    <!-- 05h : 12am - 1pm -->
    <time name="H5" duration="300" >
      <adddest name="parking"         p="0" std="0" />
      <adddest name="office-1"        p="0.05" std="0" />
      <adddest name="office-2"        p="0.05" std="0" />
      <adddest name="toilet-1"        p="0.05" std="0" />
      <adddest name="toilet-2"        p="0.05" std="0" />
      <adddest name="wa-1-l"          p="0" std="0"/>
      <adddest name="wa-1-c"          p="0" std="0"/>
      <adddest name="wa-1-r"          p="0" std="0"/>
      <adddest name="wa-2-l"          p="0" std="0"/>
      <adddest name="wa-2-c"          p="0" std="0"/>
      <adddest name="wa-2-r"          p="0" std="0"/>
      <adddest name="wa-3-l"          p="0" std="0"/>
      <adddest name="wa-3-c"          p="0.38" std="0"/>
      <adddest name="wa-3-r"          p="0" std="0"/>
      <adddest name="wa-4-l"          p="0" std="0"/>
      <adddest name="wa-4-c"          p="0" std="0"/>
      <adddest name="wa-4-r"          p="0.38" std="0"/>
      <adddest name="wa-5-l"          p="0" std="0"/>
      <adddest name="wa-5-c"          p="0" std="0"/>
      <adddest name="wa-5-r"          p="0" std="0"/>
      <adddest name="target-1"        p="0" std="0"/>
      <adddest name="target-2"        p="0" std="0"/>
      <adddest name="target-3"        p="0" std="0"/>
      <adddest name="target-4"        p="0" std="0"/>
      <adddest name="target-5"        p="0" std="0"/>
      <adddest name="target-6"        p="0" std="0"/>
      <adddest name="target-7"        p="0" std="0"/>
      <adddest name="table-12"        p="0.01" std="0" />
      <adddest name="table-23"        p="0.01" std="0" />
      <adddest name="corridor-canteen-2"       p="0.01" std="0" />
      <adddest name="corridor-canteen-3"       p="0.01" std="0" />
    </time>
    <!-- 06h : 1pm - 2pm -->
    <time name="H6" duration="300" >
      <adddest name="parking"         p="0" std="0" />
      <adddest name="office-1"        p="0" std="0" />
      <adddest name="office-2"        p="0" std="0" />
      <adddest name="wa-1-l"          p="0" std="0"/>
      <adddest name="wa-1-c"          p="0" std="0"/>
      <adddest name="wa-1-r"          p="0" std="0"/>
      <adddest name="wa-2-l"          p="0" std="0"/>
      <adddest name="wa-2-c"          p="0" std="0"/>
      <adddest name="wa-2-r"          p="0" std="0"/>
      <adddest name="wa-3-l"          p="0" std="0"/>
      <adddest name="wa-3-c"          p="0" std="0"/>
      <adddest name="wa-3-r"          p="0" std="0"/>
      <adddest name="wa-4-l"          p="0" std="0"/>
      <adddest name="wa-4-c"          p="0" std="0"/>
      <adddest name="wa-4-r"          p="0" std="0"/>
      <adddest name="wa-5-l"          p="0" std="0"/>
      <adddest name="wa-5-c"          p="0" std="0"/>
      <adddest name="wa-5-r"          p="0" std="0"/>
      <adddest name="target-1"        p="0" std="0"/>
      <adddest name="target-2"        p="0" std="0"/>
      <adddest name="target-3"        p="0" std="0"/>
      <adddest name="target-4"        p="0" std="0"/>
      <adddest name="target-5"        p="0" std="0"/>
      <adddest name="target-6"        p="0" std="0"/>
      <adddest name="target-7"        p="0" std="0"/>
      <adddest name="toilet-1"        p="0.05" std="0" />
      <adddest name="toilet-2"        p="0.05" std="0" />
      <adddest name="table-12"        p="0.225" std="0" />
      <adddest name="table-23"        p="0.225" std="0" />
      <adddest name="corridor-canteen-2"       p="0.225" std="0" />
      <adddest name="corridor-canteen-3"       p="0.225" std="0" />
    </time>
    <!-- 07h : 2pmm - 3pm -->
    <time name="H7" duration="300" >
      <adddest name="parking"         p="0" std="0" />
      <adddest name="office-1"        p="0.05" std="0" />
      <adddest name="office-2"        p="0.05" std="0" />
      <adddest name="toilet-1"        p="0.05" std="0" />
      <adddest name="toilet-2"        p="0.05" std="0" />
      <adddest name="wa-1-l"          p="0" std="0"/>
      <adddest name="wa-1-c"          p="0" std="0"/>
      <adddest name="wa-1-r"          p="0" std="0"/>
      <adddest name="wa-2-l"          p="0" std="0"/>
      <adddest name="wa-2-c"          p="0" std="0"/>
      <adddest name="wa-2-r"          p="0" std="0"/>
      <adddest name="wa-3-l"          p="0" std="0"/>
      <adddest name="wa-3-c"          p="0" std="0"/>
      <adddest name="wa-3-r"          p="0" std="0"/>
      <adddest name="wa-4-l"          p="0" std="0"/>
      <adddest name="wa-4-c"          p="0.38" std="0"/>
      <adddest name="wa-4-r"          p="0" std="0"/>
      <adddest name="wa-5-l"          p="0.38" std="0"/>
      <adddest name="wa-5-c"          p="0" std="0"/>
      <adddest name="wa-5-r"          p="0" std="0"/>
      <adddest name="target-1"        p="0" std="0"/>
      <adddest name="target-2"        p="0" std="0"/>
      <adddest name="target-3"        p="0" std="0"/>
      <adddest name="target-4"        p="0" std="0"/>
      <adddest name="target-5"        p="0" std="0"/>
      <adddest name="target-6"        p="0" std="0"/>
      <adddest name="target-7"        p="0" std="0"/>
      <adddest name="table-12"        p="0.01" std="0" />
      <adddest name="table-23"        p="0.01" std="0" />
      <adddest name="corridor-canteen-2"       p="0.01" std="0" />
      <adddest name="corridor-canteen-3"       p="0.01" std="0" />
    </time>
    <!-- 08h : 3pm - 4pm -->
    <time name="H8" duration="300" >
      <adddest name="parking"         p="0" std="0" />
      <adddest name="office-1"        p="0.05" std="0" />
      <adddest name="office-2"        p="0.05" std="0" />
      <adddest name="toilet-1"        p="0.05" std="0" />
      <adddest name="toilet-2"        p="0.05" std="0" />
      <adddest name="wa-1-l"          p="0" std="0"/>
      <adddest name="wa-1-c"          p="0.38" std="0"/>
      <adddest name="wa-1-r"          p="0" std="0"/>
      <adddest name="wa-2-l"          p="0" std="0"/>
      <adddest name="wa-2-c"          p="0" std="0"/>
      <adddest name="wa-2-r"          p="0.38" std="0"/>
      <adddest name="wa-3-l"          p="0" std="0"/>
      <adddest name="wa-3-c"          p="0" std="0"/>
      <adddest name="wa-3-r"          p="0" std="0"/>
      <adddest name="wa-4-l"          p="0" std="0"/>
      <adddest name="wa-4-c"          p="0" std="0"/>
      <adddest name="wa-4-r"          p="0" std="0"/>
      <adddest name="wa-5-l"          p="0" std="0"/>
      <adddest name="wa-5-c"          p="0" std="0"/>
      <adddest name="wa-5-r"          p="0" std="0"/>
      <adddest name="target-1"        p="0" std="0"/>
      <adddest name="target-2"        p="0" std="0"/>
      <adddest name="target-3"        p="0" std="0"/>
      <adddest name="target-4"        p="0" std="0"/>
      <adddest name="target-5"        p="0" std="0"/>
      <adddest name="target-6"        p="0" std="0"/>
      <adddest name="target-7"        p="0" std="0"/>
      <adddest name="table-12"        p="0.01" std="0" />
      <adddest name="table-23"        p="0.01" std="0" />
      <adddest name="corridor-canteen-2"       p="0.01" std="0" />
      <adddest name="corridor-canteen-3"       p="0.01" std="0" />
    </time>
    <!-- 09h : 4pm - 5pm -->
    <time name="H9" duration="300" >
      <adddest name="parking"         p="0" std="0" />
      <adddest name="office-1"        p="0.05" std="0" />
      <adddest name="office-2"        p="0.05" std="0" />
      <adddest name="toilet-1"        p="0.05" std="0" />
      <adddest name="toilet-2"        p="0.05" std="0" />
      <adddest name="wa-1-l"          p="0" std="0"/>
      <adddest name="wa-1-c"          p="0.38" std="0"/>
      <adddest name="wa-1-r"          p="0" std="0"/>
      <adddest name="wa-2-l"          p="0.38" std="0"/>
      <adddest name="wa-2-c"          p="0" std="0"/>
      <adddest name="wa-2-r"          p="0" std="0"/>
      <adddest name="wa-3-l"          p="0" std="0"/>
      <adddest name="wa-3-c"          p="0" std="0"/>
      <adddest name="wa-3-r"          p="0" std="0"/>
      <adddest name="wa-4-l"          p="0" std="0"/>
      <adddest name="wa-4-c"          p="0" std="0"/>
      <adddest name="wa-4-r"          p="0" std="0"/>
      <adddest name="wa-5-l"          p="0" std="0"/>
      <adddest name="wa-5-c"          p="0" std="0"/>
      <adddest name="wa-5-r"          p="0" std="0"/>
      <adddest name="target-1"        p="0" std="0"/>
      <adddest name="target-2"        p="0" std="0"/>
      <adddest name="target-3"        p="0" std="0"/>
      <adddest name="target-4"        p="0" std="0"/>
      <adddest name="target-5"        p="0" std="0"/>
      <adddest name="target-6"        p="0" std="0"/>
      <adddest name="target-7"        p="0" std="0"/>
      <adddest name="table-12"        p="0.01" std="0" />
      <adddest name="table-23"        p="0.01" std="0" />
      <adddest name="corridor-canteen-2"       p="0.01" std="0" />
      <adddest name="corridor-canteen-3"       p="0.01" std="0" />
    </time>
    <!-- 10h : 5pm - 6pm -->
    <time name="H10" duration="300" >
      <adddest name="parking"         p="0" std="0" />
      <adddest name="office-1"        p="0.05" std="0" />
      <adddest name="office-2"        p="0.05" std="0" />
      <adddest name="toilet-1"        p="0.05" std="0" />
      <adddest name="toilet-2"        p="0.05" std="0" />
      <adddest name="wa-1-l"          p="0" std="0"/>
      <adddest name="wa-1-c"          p="0" std="0"/>
      <adddest name="wa-1-r"          p="0" std="0"/>
      <adddest name="wa-2-l"          p="0" std="0"/>
      <adddest name="wa-2-c"          p="0" std="0"/>
      <adddest name="wa-2-r"          p="0" std="0"/>
      <adddest name="wa-3-l"          p="0" std="0"/>
      <adddest name="wa-3-c"          p="0.76" std="0"/>
      <adddest name="wa-3-r"          p="0" std="0"/>
      <adddest name="wa-4-l"          p="0" std="0"/>
      <adddest name="wa-4-c"          p="0" std="0"/>
      <adddest name="wa-4-r"          p="0" std="0"/>
      <adddest name="wa-5-l"          p="0" std="0"/>
      <adddest name="wa-5-c"          p="0" std="0"/>
      <adddest name="wa-5-r"          p="0" std="0"/>
      <adddest name="target-1"        p="0" std="0"/>
      <adddest name="target-2"        p="0" std="0"/>
      <adddest name="target-3"        p="0" std="0"/>
      <adddest name="target-4"        p="0" std="0"/>
      <adddest name="target-5"        p="0" std="0"/>
      <adddest name="target-6"        p="0" std="0"/>
      <adddest name="target-7"        p="0" std="0"/>
      <adddest name="table-12"        p="0.01" std="0" />
      <adddest name="table-23"        p="0.01" std="0" />
      <adddest name="corridor-canteen-2"       p="0.01" std="0" />
      <adddest name="corridor-canteen-3"       p="0.01" std="0" />
    </time>
    <!-- 14h : 6pm - 8am -->
    <time name="off" duration="50400" >
      <adddest name="parking"         p="1" std="0" />
      <adddest name="office-1"        p="0" std="0" />
      <adddest name="office-2"        p="0" std="0" />
      <adddest name="wa-1-l"          p="0" std="0"/>
      <adddest name="wa-1-c"          p="0" std="0"/>
      <adddest name="wa-1-r"          p="0" std="0"/>
      <adddest name="wa-2-l"          p="0" std="0"/>
      <adddest name="wa-2-c"          p="0" std="0"/>
      <adddest name="wa-2-r"          p="0" std="0"/>
      <adddest name="wa-3-l"          p="0" std="0"/>
      <adddest name="wa-3-c"          p="0" std="0"/>
      <adddest name="wa-3-r"          p="0" std="0"/>
      <adddest name="wa-4-l"          p="0" std="0"/>
      <adddest name="wa-4-c"          p="0" std="0"/>
      <adddest name="wa-4-r"          p="0" std="0"/>
      <adddest name="wa-5-l"          p="0" std="0"/>
      <adddest name="wa-5-c"          p="0" std="0"/>
      <adddest name="wa-5-r"          p="0" std="0"/>
      <adddest name="target-1"        p="0" std="0"/>
      <adddest name="target-2"        p="0" std="0"/>
      <adddest name="target-3"        p="0" std="0"/>
      <adddest name="target-4"        p="0" std="0"/>
      <adddest name="target-5"        p="0" std="0"/>
      <adddest name="target-6"        p="0" std="0"/>
      <adddest name="target-7"        p="0" std="0"/>
      <adddest name="toilet-1"        p="0" std="0" />
      <adddest name="toilet-2"        p="0" std="0" />
      <adddest name="table-12"        p="0" std="0" />
      <adddest name="table-23"        p="0" std="0" />
      <adddest name="corridor-canteen-2"       p="0" std="0" />
      <adddest name="corridor-canteen-3"       p="0" std="0" />
    </time>
  </schedule>
  """

# Parse XML
root = ET.fromstring(xml_data)

waypoints = ['canteen-2', 'canteen-3', 'table-12', 'table-23', 'wa-4-c', 'wa-2-c', 'toilet-1', 'toilet-2', 'office-1', 'office-2']
for ts in root.findall("time"):
    # Extract waypoint names and probabilities
    probabilities = []
    for dest in ts.findall("adddest"):
        name = dest.get("name")
        if name == 'corridor-canteen-2':
          name = 'canteen-2'
        elif name == 'corridor-canteen-3':
          name = 'canteen-3'
        if name in waypoints:
          prob = float(dest.get("p"))
          probabilities.append(prob)

    # Plot the probability distribution
    plt.figure(figsize=(15, 7))
    plt.barh(waypoints, probabilities, color='tab:blue')
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    plt.xlabel("Probability", fontsize=20)
    plt.ylabel("Waypoints", fontsize=20)
    plt.xlim(0, 0.4)
    tsname = ts.get('name')
    if tsname != 'off':
        tsname = tsname[1:]
    plt.title(f"Probability Distribution of Waypoints in S = {tsname}", fontsize=20)
    # plt.gca().invert_yaxis()  # Invert y-axis for better readability
    plt.grid(axis="x", linestyle="--", alpha=0.7)

    # Show plot
    plt.savefig("/home/lcastri/git/PeopleFlow/pd/"+ f"PD_{ts.get('name')}.pdf")
# for ts in root.findall("time"):
#     # Extract waypoint names and probabilities
#     waypoints = []
#     probabilities = []
#     for dest in ts.findall("adddest"):
#         name = dest.get("name")
#         prob = float(dest.get("p"))
#         if prob > 0:  # Ignore zero-probability waypoints
#           if name == 'corridor-canteen-2':
#             name = 'canteen-2'
#           elif name == 'corridor-canteen-3':
#             name = 'canteen-3'
#           # waypoints.append(name)
#           probabilities.append(prob)

#     # Plot the probability distribution
#     plt.figure(figsize=(15, 7))
#     plt.barh(waypoints, probabilities, color='tab:blue')
#     plt.xticks(fontsize=20)
#     plt.yticks(fontsize=20)
#     plt.xlabel("Probability", fontsize=20)
#     plt.ylabel("Waypoints", fontsize=20)
#     tsname = ts.get('name')
#     if tsname != 'off':
#         tsname = tsname[1:]
#     plt.title(f"Probability Distribution of Waypoints in S = {tsname}", fontsize=20)
#     # plt.gca().invert_yaxis()  # Invert y-axis for better readability
#     plt.grid(axis="x", linestyle="--", alpha=0.7)

#     # Show plot
#     plt.savefig("/home/lcastri/git/PeopleFlow/pd/"+ f"PD_{ts.get('name')}.pdf")