sudo cp build/libattach_detach.so /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libattach_detach.so
sudo rm /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libElevatorPlugin.so
sudo ln -s ~/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor/install/elevator_plugin/lib/libamr_elevator_trigger_plugin.so \
   /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/


sudo ln -s ~/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor/install/elevator_plugin/lib/libbox_spawner_plugin.so \
   /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/
