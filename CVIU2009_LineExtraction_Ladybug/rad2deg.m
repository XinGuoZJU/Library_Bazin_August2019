% used in the case the mapping toolbox is not available/installed (it has a built-in rad2deg function)
function degrees = rad2deg(radians)
degrees = radians/(pi/180);