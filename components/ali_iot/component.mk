#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)


#
# Component Makefile
#

ifdef CONFIG_ALI_IOT
# eng/coap_server eng/coap_server/CoAPPacket eng/coap_server/server 

COMPONENT_ADD_INCLUDEDIRS := eng \
	eng/dev_model \
	eng/dev_sign \
	eng/infra \
	eng/mqtt \
	eng/wrappers \
	eng/wrappers/external_libs \
	examples
#	eng/dev_bind/impl \
#	eng/dev_bind/impl/awss_reset \
#	eng/dev_bind/impl/os \
#	eng/wifi_provision \
#	eng/wifi_provision/frameworks \
#	eng/wifi_provision/frameworks \
#	eng/wifi_provision/frameworks/aplist \
#	eng/wifi_provision/frameworks/ieee80211 \
#	eng/wifi_provision/frameworks/statics \
#	eng/wifi_provision/frameworks/utils \
#	eng/wifi_provision/p2p \
#	eng/wifi_provision/smartconfig \
#	eng/ota eng/wrappers \



COMPONENT_SRCDIRS := eng \
	eng/dev_model \
	eng/dev_sign \
	eng/infra \
	eng/mqtt \
	eng/wrappers \
	eng/wrappers/external_libs \
	examples
#	eng/dev_bind/impl \
#	eng/dev_bind/impl/awss_reset \
#	eng/dev_bind/impl/os \
#	eng/wifi_provision \
#	eng/wifi_provision/frameworks \
#	eng/wifi_provision/frameworks \
#	eng/wifi_provision/frameworks/aplist \
#	eng/wifi_provision/frameworks/ieee80211 \
#	eng/wifi_provision/frameworks/statics \
#	eng/wifi_provision/frameworks/utils \
#	eng/wifi_provision/p2p \
#	eng/wifi_provision/smartconfig \
#	eng/ota eng/wrappers \

# Check the submodule is initialised
COMPONENT_SUBMODULES := 


else
# Disable AWS IoT support
COMPONENT_ADD_INCLUDEDIRS :=
COMPONENT_ADD_LDFLAGS :=
COMPONENT_SRCDIRS :=
endif
