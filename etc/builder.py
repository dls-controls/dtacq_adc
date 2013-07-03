from iocbuilder import Device, AutoSubstitution, SetSimulation
from iocbuilder.arginfo import *

from iocbuilder.modules.areaDetector import AreaDetector, _ADBase, _ADBaseTemplate, _NDPluginProducerBase, _NDPluginBase
from iocbuilder.modules.asyn import AsynIP

class _dtacq_adc(AutoSubstitution):
    TemplateFile = "dtacq_adc.template"

class dtacq_adc(_ADBase):
    """Creates a simulation detector"""
    _SpecificTemplate = _dtacq_adc
    def __init__(self, IP, WIDTH, HEIGHT, DATATYPE = 1, BUFFERS = 50, MEMORY = 0, **args):
        self.ip = AsynIP(IP, name = args["PORT"]+"ip")    
        # Init the superclass (_ADBase)
        self.__super.__init__(**args)
        # Store the args
        self.__dict__.update(locals())

    # __init__ arguments
    ArgInfo = _ADBase.ArgInfo + _SpecificTemplate.ArgInfo + makeArgInfo(__init__,
        IP = Simple('IP Port number', str),
        WIDTH = Simple('Image Width', int),
        HEIGHT = Simple('Image Height', int),
        DATATYPE = Enum('Datatype',
                ["NDInt8", "NDUInt8", "NDInt16", "NDUInt16", "NDInt32",
                 "NDUInt32", "NDFloat32", "NDFloat64"]),
        BUFFERS = Simple('Maximum number of NDArray buffers to be created for '
            'plugin callbacks', int),
        MEMORY = Simple('Max memory to allocate, should be maxw*maxh*nbuffer '
            'for driver and all attached plugins', int))

    # Device attributes
    LibFileList = ['dtacq_adc']
    DbdFileList = ['dtacq_adcSupport']

    def Initialise(self):
        print '# dtacq_adcConfig(portName, ipPortName, maxSizeX, maxSizeY, dataType, ' \
            'maxBuffers, maxMemory)'
        print 'dtacq_adcConfig("%(PORT)s", "%(PORT)sip", %(WIDTH)s, %(HEIGHT)s, %(DATATYPE)d, ' \
            '%(BUFFERS)d, %(MEMORY)d)' % self.__dict__
            
class _BPM_calc(AutoSubstitution):
    """Template containing the records for an NDROI"""
    TemplateFile = 'BPM_calc.template'

class BPM_calc(_NDPluginProducerBase):
    """This plugin selects a region of interest and optionally scales it to
    fit in a particular data type"""
    _SpecificTemplate = _BPM_calc
    # NOTE: _NDPluginBase comes 2nd so we overwrite NDARRAY_PORT argInfo
    ArgInfo = _SpecificTemplate.ArgInfo + _NDPluginBase.ArgInfo

class _FFT_calc(AutoSubstitution):
    TemplateFile = 'FFT_calc.template'

class FFT_calc(_NDPluginProducerBase):
    _SpecificTemplate = _FFT_calc
    ArgIndo = _SpecificTemplate.ArgInfo + _NDPluginBase.ArgInfo
