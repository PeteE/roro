Import('env')
from base64 import b64decode

env.Replace(UPLOADHEXCMD='sudo avrdude ' + b64decode(ARGUMENTS.get("CUSTOM_OPTION")) + ' -U flash:w:$SOURCES:i')

# uncomment line below to see environment variables
#print env.Dump()
#print ARGUMENTS
