TOP=../..

include $(TOP)/configure/CONFIG

# -------------------------------
# Build an Diamond Support Module
# -------------------------------

LIBRARY_IOC += dtacq_adc

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxx.h

# xxxRecord.dbd will be installed into <top>/dbd
DBD += dtacq_adcSupport.dbd

# The following are compiled and added to the support library
dtacq_adc_SRCS += BPM_calc.cpp
dtacq_adc_SRCS += bessel.cpp
dtacq_adc_SRCS += FFT_calc.cpp
dtacq_adc_SRCS += dtacq_adc.cpp
dtacq_adc_SRCS += window.cpp
dtacq_adc_SRCS += fftpack.cpp

# We need to link against the EPICS Base libraries
#xxx_LIBS += $(EPICS_BASE_IOC_LIBS)

USR_CXXFLAGS = -ggdb -O0

include $(TOP)/configure/RULES
