# This file is part of the REMOTE API
# 
# Copyright 2006-2017 Coppelia Robotics GmbH. All rights reserved. 
# marc@coppeliarobotics.com
# www.coppeliarobotics.com
# 
# The REMOTE API is licensed under the terms of GNU GPL:
# 
# -------------------------------------------------------------------
# The REMOTE API is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# THE REMOTE API IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
# AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
# DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
# MISUSING THIS SOFTWARE.
# 
# See the GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with the REMOTE API.  If not, see <http://www.gnu.org/licenses/>.
# -------------------------------------------------------------------
#
# This file was automatically created for V-REP release V3.4.0 rev. 1 on April 5th 2017

#constants
#Scene object types. Values are serialized
sim_object_shape_type           =0
sim_object_joint_type           =1
sim_object_graph_type           =2
sim_object_camera_type          =3
sim_object_dummy_type           =4
sim_object_proximitysensor_type =5
sim_object_reserved1            =6
sim_object_reserved2            =7
sim_object_path_type            =8
sim_object_visionsensor_type    =9
sim_object_volume_type          =10
sim_object_mill_type            =11
sim_object_forcesensor_type     =12
sim_object_light_type           =13
sim_object_mirror_type          =14

#General object types. Values are serialized
sim_appobj_object_type          =109
sim_appobj_collision_type       =110
sim_appobj_distance_type        =111
sim_appobj_simulation_type      =112
sim_appobj_ik_type              =113
sim_appobj_constraintsolver_type=114
sim_appobj_collection_type      =115
sim_appobj_ui_type              =116
sim_appobj_script_type          =117
sim_appobj_pathplanning_type    =118
sim_appobj_RESERVED_type        =119
sim_appobj_texture_type         =120

# Ik calculation methods. Values are serialized
sim_ik_pseudo_inverse_method        =0
sim_ik_damped_least_squares_method  =1
sim_ik_jacobian_transpose_method    =2

# Ik constraints. Values are serialized
sim_ik_x_constraint         =1
sim_ik_y_constraint         =2
sim_ik_z_constraint         =4
sim_ik_alpha_beta_constraint=8
sim_ik_gamma_constraint     =16
sim_ik_avoidance_constraint =64

# Ik calculation results 
sim_ikresult_not_performed  =0
sim_ikresult_success        =1
sim_ikresult_fail           =2

# Scene object sub-types. Values are serialized 
# Light sub-types 
sim_light_omnidirectional_subtype   =1
sim_light_spot_subtype              =2
sim_light_directional_subtype       =3
# Joint sub-types 
sim_joint_revolute_subtype          =10
sim_joint_prismatic_subtype         =11
sim_joint_spherical_subtype         =12
# Shape sub-types 
sim_shape_simpleshape_subtype       =20
sim_shape_multishape_subtype        =21
# Proximity sensor sub-types 
sim_proximitysensor_pyramid_subtype =30
sim_proximitysensor_cylinder_subtype=31
sim_proximitysensor_disc_subtype    =32
sim_proximitysensor_cone_subtype    =33
sim_proximitysensor_ray_subtype     =34
# Mill sub-types 
sim_mill_pyramid_subtype            =40
sim_mill_cylinder_subtype           =41
sim_mill_disc_subtype               =42
sim_mill_cone_subtype               =42
# No sub-type 
sim_object_no_subtype               =200


#Scene object main properties (serialized)
sim_objectspecialproperty_collidable                    =0x0001
sim_objectspecialproperty_measurable                    =0x0002
#reserved                        =0x0004 
#reserved                        =0x0008 
sim_objectspecialproperty_detectable_ultrasonic            =0x0010
sim_objectspecialproperty_detectable_infrared            =0x0020
sim_objectspecialproperty_detectable_laser                =0x0040
sim_objectspecialproperty_detectable_inductive            =0x0080
sim_objectspecialproperty_detectable_capacitive            =0x0100
sim_objectspecialproperty_renderable                    =0x0200
sim_objectspecialproperty_detectable_all =sim_objectspecialproperty_detectable_ultrasonic|sim_objectspecialproperty_detectable_infrared|sim_objectspecialproperty_detectable_laser|sim_objectspecialproperty_detectable_inductive|sim_objectspecialproperty_detectable_capacitive
sim_objectspecialproperty_cuttable                        =0x0400
sim_objectspecialproperty_pathplanning_ignored            =0x0800

# Model properties (serialized)
sim_modelproperty_not_collidable                =0x0001
sim_modelproperty_not_measurable                =0x0002
sim_modelproperty_not_renderable                =0x0004
sim_modelproperty_not_detectable                =0x0008
sim_modelproperty_not_cuttable                    =0x0010
sim_modelproperty_not_dynamic                    =0x0020
sim_modelproperty_not_respondable                =0x0040 # cannot be selected if sim_modelproperty_not_dynamic is not selected 
sim_modelproperty_not_reset                        =0x0080 # Model is not reset at simulation end. This flag is cleared at simulation end 
sim_modelproperty_not_visible                    =0x0100 # Whole model is invisible independent of local visibility settings 
sim_modelproperty_not_model                        =0xf000 # object is not a model 


# Check the documentation instead of comments below!! 
# Following messages are dispatched to the Lua-message container 
sim_message_ui_button_state_change  =0    # a UI button slider etc. changed (due to a user's action). aux[0]=UI handle aux[1]=button handle aux[2]=button attributes aux[3]=slider position (if slider) 
sim_message_reserved9               =1    # Do not use 
sim_message_object_selection_changed=2
sim_message_reserved10                =3    # do not use 
sim_message_model_loaded            =4
sim_message_reserved11                =5    # do not use 
sim_message_keypress                =6    # a key was pressed while the focus was on a page (aux[0]=key aux[1]=ctrl and shift key state) 
sim_message_bannerclicked            =7    # a banner was clicked (aux[0]=banner ID) 


# Following messages are dispatched only to the C-API (not available from Lua) 
sim_message_for_c_api_only_start        =0x100      # Do not use 
sim_message_reserved1                   =0x101      # Do not use 
sim_message_reserved2                    =0x102      # Do not use 
sim_message_reserved3                    =0x103      # Do not use 
sim_message_eventcallback_scenesave        =0x104        # about to save a scene 
sim_message_eventcallback_modelsave        =0x105      # about to save a model (current selection will be saved) 
sim_message_eventcallback_moduleopen    =0x106        # called when simOpenModule in Lua is called 
sim_message_eventcallback_modulehandle    =0x107        # called when simHandleModule in Lua is called with argument false 
sim_message_eventcallback_moduleclose    =0x108        # called when simCloseModule in Lua is called 
sim_message_reserved4                    =0x109      # Do not use 
sim_message_reserved5                    =0x10a        # Do not use 
sim_message_reserved6                    =0x10b        # Do not use 
sim_message_reserved7                    =0x10c        # Do not use 
sim_message_eventcallback_instancepass    =0x10d        # Called once every main application loop pass. auxiliaryData[0] contains event flags of events that happened since last time 
sim_message_eventcallback_broadcast     =0x10e
sim_message_eventcallback_imagefilter_enumreset =0x10f
sim_message_eventcallback_imagefilter_enumerate      =0x110
sim_message_eventcallback_imagefilter_adjustparams   =0x111
sim_message_eventcallback_imagefilter_reserved       =0x112
sim_message_eventcallback_imagefilter_process        =0x113
sim_message_eventcallback_reserved1                  =0x114   # do not use 
sim_message_eventcallback_reserved2                  =0x115   # do not use 
sim_message_eventcallback_reserved3                  =0x116   # do not use 
sim_message_eventcallback_reserved4                  =0x117   # do not use 
sim_message_eventcallback_abouttoundo                 =0x118   # the undo button was hit and a previous state is about to be restored 
sim_message_eventcallback_undoperformed                 =0x119   # the undo button was hit and a previous state restored 
sim_message_eventcallback_abouttoredo                 =0x11a   # the redo button was hit and a future state is about to be restored  
sim_message_eventcallback_redoperformed                 =0x11b   # the redo button was hit and a future state restored  
sim_message_eventcallback_scripticondblclick         =0x11c   # scipt icon was double clicked.  (aux[0]=object handle associated with script set replyData[0] to 1 if script should not be opened)  
sim_message_eventcallback_simulationabouttostart     =0x11d
sim_message_eventcallback_simulationended            =0x11e
sim_message_eventcallback_reserved5                     =0x11f   # do not use 
sim_message_eventcallback_keypress                     =0x120   # a key was pressed while the focus was on a page (aux[0]=key aux[1]=ctrl and shift key state) 
sim_message_eventcallback_modulehandleinsensingpart  =0x121   # called when simHandleModule in Lua is called with argument true 
sim_message_eventcallback_renderingpass              =0x122   # called just before the scene is rendered 
sim_message_eventcallback_bannerclicked              =0x123   # called when a banner was clicked (aux[0]=banner ID) 
sim_message_eventcallback_menuitemselected           =0x124   # auxiliaryData[0] indicates the handle of the item auxiliaryData[1] indicates the state of the item 
sim_message_eventcallback_refreshdialogs             =0x125   # aux[0]=refresh degree (0=light 1=medium 2=full) 
sim_message_eventcallback_sceneloaded                =0x126
sim_message_eventcallback_modelloaded                =0x127
sim_message_eventcallback_instanceswitch             =0x128
sim_message_eventcallback_guipass                    =0x129
sim_message_eventcallback_mainscriptabouttobecalled  =0x12a
sim_message_eventcallback_rmlposition                =0x12b   #the command simRMLPosition was called. The appropriate plugin should handle the call
sim_message_eventcallback_rmlvelocity                =0x12c   # the command simRMLVelocity was called. The appropriate plugin should handle the call
sim_message_simulation_start_resume_request          =0x1000
sim_message_simulation_pause_request                 =0x1001
sim_message_simulation_stop_request                  =0x1002

# Scene object properties. Combine with the | operator 
sim_objectproperty_reserved1                =0x0000
sim_objectproperty_reserved2                =0x0001
sim_objectproperty_reserved3                =0x0002
sim_objectproperty_reserved4                =0x0003
sim_objectproperty_reserved5                =0x0004 # formely sim_objectproperty_visible 
sim_objectproperty_reserved6                =0x0008 # formely sim_objectproperty_wireframe 
sim_objectproperty_collapsed                =0x0010
sim_objectproperty_selectable                =0x0020
sim_objectproperty_reserved7                =0x0040
sim_objectproperty_selectmodelbaseinstead    =0x0080
sim_objectproperty_dontshowasinsidemodel    =0x0100
# reserved                                    =0x0200 
sim_objectproperty_canupdatedna                =0x0400
sim_objectproperty_selectinvisible            =0x0800
sim_objectproperty_depthinvisible            =0x1000


# type of arguments (input and output) for custom lua commands 
sim_lua_arg_nil     =0
sim_lua_arg_bool    =1    
sim_lua_arg_int     =2
sim_lua_arg_float   =3
sim_lua_arg_string  =4
sim_lua_arg_invalid =5
sim_lua_arg_table   =8

# custom user interface properties. Values are serialized. 
sim_ui_property_visible                        =0x0001
sim_ui_property_visibleduringsimulationonly    =0x0002
sim_ui_property_moveable                    =0x0004
sim_ui_property_relativetoleftborder        =0x0008
sim_ui_property_relativetotopborder            =0x0010
sim_ui_property_fixedwidthfont                =0x0020
sim_ui_property_systemblock                    =0x0040
sim_ui_property_settocenter                    =0x0080
sim_ui_property_rolledup                    =0x0100
sim_ui_property_selectassociatedobject        =0x0200
sim_ui_property_visiblewhenobjectselected    =0x0400


# button properties. Values are serialized. 
sim_buttonproperty_button                =0x0000
sim_buttonproperty_label                =0x0001
sim_buttonproperty_slider                =0x0002
sim_buttonproperty_editbox                =0x0003
sim_buttonproperty_staydown                =0x0008
sim_buttonproperty_enabled                =0x0010
sim_buttonproperty_borderless            =0x0020
sim_buttonproperty_horizontallycentered    =0x0040
sim_buttonproperty_ignoremouse            =0x0080
sim_buttonproperty_isdown                =0x0100
sim_buttonproperty_transparent            =0x0200
sim_buttonproperty_nobackgroundcolor    =0x0400
sim_buttonproperty_rollupaction            =0x0800
sim_buttonproperty_closeaction            =0x1000
sim_buttonproperty_verticallycentered    =0x2000
sim_buttonproperty_downupevent            =0x4000


# Simulation status 
sim_simulation_stopped                        =0x00                                # Simulation is stopped 
sim_simulation_paused                        =0x08                                # Simulation is paused 
sim_simulation_advancing                    =0x10                                # Simulation is advancing 
sim_simulation_advancing_firstafterstop        =sim_simulation_advancing|0x00        # First simulation pass (1x) 
sim_simulation_advancing_running            =sim_simulation_advancing|0x01        # Normal simulation pass (>=1x) 
# reserved                                    =sim_simulation_advancing|0x02 
sim_simulation_advancing_lastbeforepause    =sim_simulation_advancing|0x03        # Last simulation pass before pause (1x) 
sim_simulation_advancing_firstafterpause    =sim_simulation_advancing|0x04        # First simulation pass after pause (1x) 
sim_simulation_advancing_abouttostop        =sim_simulation_advancing|0x05        # "Trying to stop" simulation pass (>=1x) 
sim_simulation_advancing_lastbeforestop        =sim_simulation_advancing|0x06        # Last simulation pass (1x) 


# Script execution result (first return value) 
sim_script_no_error                    =0
sim_script_main_script_nonexistent    =1
sim_script_main_script_not_called    =2
sim_script_reentrance_error            =4
sim_script_lua_error                =8
sim_script_call_error                =16


 # Script types (serialized!) 
sim_scripttype_mainscript   =0
sim_scripttype_childscript  =1
sim_scripttype_jointctrlcallback  =4
sim_scripttype_contactcallback  =5
sim_scripttype_customizationscript  =6
sim_scripttype_generalcallback  =7

# API call error messages 
sim_api_errormessage_ignore    =0    # does not memorize nor output errors 
sim_api_errormessage_report    =1    # memorizes errors (default for C-API calls) 
sim_api_errormessage_output    =2  # memorizes and outputs errors (default for Lua-API calls) 


# special argument of some functions 
sim_handle_all                        =-2
sim_handle_all_except_explicit        =-3
sim_handle_self                        =-4
sim_handle_main_script                =-5
sim_handle_tree                        =-6
sim_handle_chain                    =-7
sim_handle_single                    =-8
sim_handle_default                    =-9
sim_handle_all_except_self            =-10
sim_handle_parent                    =-11


# special handle flags
sim_handleflag_assembly                =0x400000
sim_handleflag_model                =0x800000


# distance calculation methods (se