set(SRC_CPP_FILES

)

set(INTERNAL_CPP_FILES
  org_mitk_lancet_dentalrobot_Activator.cpp
  DentalRobot.cpp
  DentalRobot_navigation.cpp
  DentalRobot_planning.cpp
  DentalRobot_activate.cpp
  DentalRobot_robotarmRegistration.cpp
  PrintDataHelper.cpp
  DentalRobot_imageRegistration.cpp
  ProbeInteractor.cpp
)

set(UI_FILES
  src/internal/DentalRobotControls.ui
)

set(MOC_H_FILES
  src/internal/org_mitk_lancet_dentalrobot_Activator.h
  src/internal/DentalRobot.h
  src/internal/PrintDataHelper.h
  src/internal/ProbeInteractor.h
)

# list of resource files which can be used by the plug-in
# system without loading the plug-ins shared library,
# for example the icon used in the menu and tabs for the
# plug-in views in the workbench
set(CACHED_RESOURCE_FILES
  resources/icon.xpm
  plugin.xml
)

# list of Qt .qrc files which contain additional resources
# specific to this plugin
set(QRC_FILES

)

set(CPP_FILES )

foreach(file ${SRC_CPP_FILES})
  set(CPP_FILES ${CPP_FILES} src/${file})
endforeach(file ${SRC_CPP_FILES})

foreach(file ${INTERNAL_CPP_FILES})
  set(CPP_FILES ${CPP_FILES} src/internal/${file})
endforeach(file ${INTERNAL_CPP_FILES})
