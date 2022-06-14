# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set (WBK_PROJECT_NAME WholeBodyKinematicsDevice)

# HDE xml files installation
file (GLOB WBK_XML_FILES xml/*.xml)

install(FILES ${WBK_XML_FILES}
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${WBK_PROJECT_NAME}/)

