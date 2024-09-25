#!/usr/bin/env python

import sys

from rqt_peopleflow.src.rqt_peopleflow.peopleflow_monitor import PeopleflowPlugin
from rqt_gui.main import Main

plugin = 'rqt_peopleflow'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))