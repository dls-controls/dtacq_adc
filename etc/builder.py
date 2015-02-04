from iocbuilder import Device, AutoSubstitution, SetSimulation
from iocbuilder.arginfo import *

from iocbuilder.modules.areaDetector import AreaDetector, _ADBase, _ADBaseTemplate, _NDPluginProducerBase, _NDPluginBase
from iocbuilder.modules.asyn import AsynIP

class _dtacq_adc(AutoSubstitution):
    TemplateFile = "dtacq_adc.template"

class dtacq_adc(_ADBase):
    """Creates a simulation detector"""
    _SpecificTemplate = _dtacq_adc
    def __init__(self, DATA_IP, CONTROL_IP, NCHANNELS=4, NSAMPLES=1000000,
                 BUFFERS=50, MEMORY=0, **args):
        self.controlPort = AsynIP(CONTROL_IP, name = args["PORT"] + ".control")
        # Init the superclass (_ADBase)
        self.__super.__init__(**args)
        # Store the args
        self.__dict__.update(locals())

    # __init__ arguments
    ArgInfo = _ADBase.ArgInfo + _SpecificTemplate.ArgInfo + makeArgInfo(
        __init__,
        DATA_IP = Simple('Address and data port number', str),
        CONTROL_IP = Simple('Address and control port number', str),
        NCHANNELS = Simple('Number of channels on the ADC', int),
        NSAMPLES = Simple('Number of samples from the ADC to create an NDArray',
                          int),
        BUFFERS = Simple('Maximum number of NDArray buffers to be created for '
                         'plugin callbacks', int),
        MEMORY = Simple('Max memory to allocate, should be maxw*maxh*nbuffer '
                        'for driver and all attached plugins', int))

    # Device attributes
    LibFileList = ['dtacq_adc']
    DbdFileList = ['dtacq_adcSupport']

    def Initialise(self):
        print(
'''# dtacq_adcConfig(portName, dataPortName, controlPortName,
#                 nChannels, nSamples, maxBuffers, maxMemory,
#                 dataHostInfo)''')
        print('''dtacq_adcConfig("%(PORT)s", "%(PORT)s.data", \
"%(PORT)s.control", %(NCHANNELS)d, %(NSAMPLES)d, \
%(BUFFERS)d, %(MEMORY)d, "%(DATA_IP)s")''' % self.__dict__)
