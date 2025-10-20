import rospy
from peopleflow_msgs.msg import pfAgents
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QTreeWidget, QTreeWidgetItem, QHeaderView
from rqt_gui_py.plugin import Plugin

class PeopleflowPlugin(Plugin):
    def __init__(self, context):
        super(PeopleflowPlugin, self).__init__(context)
        self.setObjectName('PeopleflowPlugin')

        self._widget = QWidget()
        self._layout = QVBoxLayout(self._widget)

        self._tree = QTreeWidget()
        self._tree.setHeaderLabel('Agents')
        self._tree.header().setSectionResizeMode(QHeaderView.ResizeToContents)

        self._layout.addWidget(self._tree)
        self._widget.setLayout(self._layout)

        context.add_widget(self._widget)

        # ROS subscriber
        self._sub = rospy.Subscriber('/peopleflow/agents', pfAgents, self._callback)

    def _callback(self, msg):
        # Step 1: Save the current expansion state
        expansion_state = {}
        self.save_expansion_state(self._tree.invisibleRootItem(), expansion_state)

        # Step 2: Update the tree (modify existing items rather than clearing and recreating the tree)
        self.update_tree_with_msg(msg)

        # Step 3: Restore the expansion state
        self.restore_expansion_state(self._tree.invisibleRootItem(), expansion_state)

    def save_expansion_state(self, item, expansion_state, path=""):
        path = path + "/" + item.text(0)
        expansion_state[path] = item.isExpanded()

        for i in range(item.childCount()):
            child = item.child(i)
            self.save_expansion_state(child, expansion_state, path)

    def restore_expansion_state(self, item, expansion_state, path=""):
        path = path + "/" + item.text(0)
        if path in expansion_state:
            item.setExpanded(expansion_state[path])

        for i in range(item.childCount()):
            child = item.child(i)
            self.restore_expansion_state(child, expansion_state, path)

    def update_tree_with_msg(self, msg):
        agents = msg.agents

        # Iterate through the existing items and update them
        for i, agent in enumerate(agents):
            if i < self._tree.topLevelItemCount():
                agent_item = self._tree.topLevelItem(i)
            else:
                agent_item = QTreeWidgetItem(self._tree)

            # Update agent details
            agent_item.setText(0, f"Agent {agent.id}")

            # Update or create child items
            self.update_or_create_child(agent_item, 0, f"ID: {agent.id}")
            self.update_or_create_child(agent_item, 1, f"Starting Time: {agent.starting_time.data}")
            self.update_or_create_child(agent_item, 2, f"Exit Time: {agent.exit_time.data}")
            self.update_or_create_child(agent_item, 3, f"Position: x={round(agent.position.x, 2)}, y={round(agent.position.y, 2)}")
            self.update_or_create_child(agent_item, 4, f"Is Stuck: {agent.is_stuck.data}")
            self.update_or_create_child(agent_item, 5, f"At Work: {agent.at_work.data}")
            self.update_or_create_child(agent_item, 6, f"Is Quitting: {agent.is_quitting.data}")
            self.update_or_create_child(agent_item, 7, f"Path: {' -- '.join([str(p.data) for p in agent.path])}")
            self.update_or_create_child(agent_item, 8, f"Past WP ID: {agent.past_WP_id.data}")
            self.update_or_create_child(agent_item, 9, f"Current WP ID: {agent.current_WP_id.data}")
            self.update_or_create_child(agent_item, 10, f"Next WP ID: {agent.next_WP_id.data}, x={round(agent.next_destination.x, 2)}, y={round(agent.next_destination.y, 2)}, r={agent.next_destination_radius}")
            self.update_or_create_child(agent_item, 11, f"Final WP ID: {agent.final_WP_id.data}")
            self.update_or_create_child(agent_item, 12, f"Task Duration: {agent.task_duration}")

        # Remove extra items if there are fewer agents than before
        while self._tree.topLevelItemCount() > len(agents):
            self._tree.takeTopLevelItem(self._tree.topLevelItemCount() - 1)

    def update_or_create_child(self, parent_item, child_index, text):
        # If the child exists, update its text; otherwise, create a new child item
        if child_index < parent_item.childCount():
            child_item = parent_item.child(child_index)
            child_item.setText(0, text)
        else:
            child_item = QTreeWidgetItem(parent_item)
            child_item.setText(0, text)
            parent_item.addChild(child_item)

    def shutdown_plugin(self):
        self._sub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
