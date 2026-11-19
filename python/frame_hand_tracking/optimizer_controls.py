import math


def mercury_optimizer_controls(controls):
    value=controls.get('mercury_opt_smooth_factor',controls.get('smoothing',.2))
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0<=value<=30:
        raise ValueError('Invalid mercury_opt_smooth_factor: expected finite number 0..30')
    size=controls.get('mercury_optimize_hand_size',True)
    if type(size) is not bool:
        raise ValueError('Invalid mercury_optimize_hand_size: expected boolean')
    return {'opt_smooth_factor':float(value),'optimize_hand_size':size}


def mercury_reset_signature(controls):
    parameters=mercury_optimizer_controls(controls)
    return (parameters['opt_smooth_factor'],parameters['optimize_hand_size'])
