
def term_stat(message):
    if message == '{"command":"terminate"}': return True


"""
necessary functionality:
    * terminate
    * move arm relative/absolute x, y, z
    * move arm relative/absolute y
    * move arm relative/absolute x
    * move arm relative/absolute z
    * end effector action 1
    * end effector action 2
    * 
"""

"""
# terminate
{"command":"terminate"}

# move arm to x, y, z
{
    "moveAbsolute": [
        1,
        1,
        1
    ]
}

# move arm relative x
{
    "moveRelativeX": [
        1
    ]
}

# move arm relative y
{
    "moveRelativeY": [
        1
    ]
}

# move arm relative z
{
    "moveRelativeZ": [
        1
    ]
}

"""