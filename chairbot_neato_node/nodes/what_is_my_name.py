"""
Abrar: This will return the chairbot number by extracting it from the hostname of the machine
e.g. if the hostname is chairbot03-desktop, it will return the String "03"
Last update: Feb 12, 2019
"""

import platform

def what_is_my_name():
    hostname =  platform.node()
    return hostname.split('-')[0][-2:]

def what_is_my_number():
    return what_is_my_name()

def chairbot_number():
    return what_is_my_name()

#print the value only if this is run as a script
if __name__ == "__main__":
    print(what_is_my_name())
