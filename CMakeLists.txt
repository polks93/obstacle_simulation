cmake_minimum_required(VERSION 3.0.2)
project(obstacle-simulation)

# Trova i pacchetti ROS necessari
find_package(catkin REQUIRED COMPONENTS
  rospy
  # Aggiungi qui altri pacchetti ROS se ne hai bisogno, ad esempio std_msgs, geometry_msgs, ecc.
)

# Integra il setup.py per installare i pacchetti Python
catkin_python_setup()

# Crea il pacchetto ROS
catkin_package()

# (Facoltativo) Se hai script Python da installare come eseguibili
# install(PROGRAMS
#   scripts/mio_nodo.py    # Aggiungi qui i tuoi script Python
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# (Facoltativo) Se hai file di lancio, puoi includerli qui
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
