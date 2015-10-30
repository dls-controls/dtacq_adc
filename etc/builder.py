from iocbuilder import Device, AutoSubstitution, SetSimulation
from iocbuilder.arginfo import *
from iocbuilder.modules.asyn import Asyn, AsynPort, AsynIP

from iocbuilder.modules.ADCore import ADCore, ADBaseTemplate, makeTemplateInstance, includesTemplates

@includesTemplates(ADBaseTemplate)
class _dtacq_adc(AutoSubstitution):
    TemplateFile = "dtacq_adc.template"

class dtacq_adc(AsynPort):
    """Creates a dtacq_adc driver"""
    Dependencies = (ADCore, Asyn)
    UniqueName = "PORT"
    _SpecificTemplate = _dtacq_adc
    def __init__(self, DATA_IP, CONTROL_IP, NCHANNELS=4, NSAMPLES=1000000,
                 BUFFERS=50, MEMORY=0, **args):
        self.controlPort = AsynIP(CONTROL_IP, name = args["PORT"] + ".control")
        # Init the superclass (_ADBase)
        self.__super.__init__(args["PORT"])
        # Store the args
        self.__dict__.update(locals())
        makeTemplateInstance(self._SpecificTemplate, locals(), args)

    # __init__ arguments
    ArgInfo = ADBaseTemplate.ArgInfo + _SpecificTemplate.ArgInfo + makeArgInfo(
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
        print('''dtacq_adcConfig("%s", "%s.data", \
"%s.control", %d, %d, \
%d, %d, "%s")''' % (self.__dict__["args"]["PORT"], self.__dict__["args"]["PORT"], self.__dict__["args"]["PORT"], self.__dict__["NCHANNELS"], self.__dict__["NSAMPLES"], self.__dict__["BUFFERS"], self.__dict__["MEMORY"], self.__dict__["DATA_IP"]))

