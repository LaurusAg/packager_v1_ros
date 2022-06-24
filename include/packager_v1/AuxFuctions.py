## Parse a bitfield (string) according to the mapping indicated in the dictionary 
## Dict values: indicate the size of each field of the bitfield. The bits are popped from LSB to MSB and assigned to the dictionary in order
## @return dictionary with the same keys as the original but with the values replaced with the bitfield values
## Example:
    ## status = '0000-0000-0001-1011'
    ## d = {'ls_min': 1, 'enc_seal': 1, 'enc_disp': 1, 'ls_max': 1, 'state': 3 }
    
    ## Returns -> d = {'ls_min': '1', 'enc_seal': '1', 'enc_disp': '0', 'ls_max': '1', 'state': '001'}
def ParseBitfield(bitfield, dictionary):
    retval = dictionary ## WARNING: modifying original dictionary
    offset = 0
    bitfield = bitfield.replace('-', '')
    for key in dictionary:
        bitsize = dictionary[key]
        val = bitfield[-(offset+bitsize):-offset] if offset != 0 else bitfield[-bitsize]
        offset = offset + bitsize
        retval[key] = val
    return retval