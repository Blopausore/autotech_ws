def linear_transformation(value, old_scale, new_scale):
    alpha = (new_scale[1] - new_scale[0])/(old_scale[1] - old_scale[0]) 
    return (value - old_scale[0])*alpha + new_scale[0]