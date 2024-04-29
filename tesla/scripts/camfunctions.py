# A script with all the functions in side. This is for ease of testing.

# A Function to find and return the width in cm
# bl = bottomleft, br = bottomright
def widthFinder(bl, br):
    
    #------Both the pixels are in the same range------#
    # both below 245 pix
    if bl <= 245 and br <= 245:
        return 10
    
    # both within 246 - 705
    if bl <= 705 and bl > 245 and br <= 705:
        return 10
    
    # both within 706 - 1155
    if bl > 705 and bl <= 1155 and br <= 1155:
        return 10

    # both within 1155 - 1280
    if bl > 1155 and bl <= 1280 and br <= 1280:
            return 4
    
    #------Pixels in different ranges------#
    # only bl below 246 pix
    while bl <= 245 and br > 245:
        # br is above 245 and below 706
        if br <= 705:
            return 20
        # br is below 1155 
        elif br <= 1155:
            return 30
        # br is below 1280
        elif br <= 1280:
            return 34
        else:
            break

    # bl is above 245 and below 706 and br is above that.   
    while bl > 245 and bl <= 705 and br > 705:
        if br <= 1155:
            return 20
        
        elif br <= 1280:
            return 24
    
    # bl is above 705 and below 1156 and br is above that.
    while bl > 705 and bl <= 1155 and br > 1155:
        if br <= 1280:
            return 14
        


# Function to calculate the distance given the number of pixels
# bp = bottom pixel (the bottom of the bounding box.)
def distanceFinder(bp):
    if bp > 392:
        # object is super close
        # lets just say return 5 cm
        return 5
    elif bp <=392 and bp > 207:
        return 10
    elif bp <= 207 and bp > 121:
        return 20
    elif bp <= 121 and bp > 72:
        return 30
    elif bp <= 72 and bp > 40:
        return 40
    elif bp <= 40 and bp > 22:
        return 50
    elif bp <= 22:
        return 60
