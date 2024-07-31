# -*- coding: utf-8 -*-

# Cargar la nube de puntos


# Crear un filtro PassThrough
passthrough = cloud.make_passthrough_filter()

# Assignar el eje y el rango para el filtro.
filter_axis = 'y'
passthrough.set_filter_field_name(filter_axis)
min_val = 1.0
max_val = 1.5
passthrough.set_filter_limits(min_val, max_val)

# Usar el filtro para obtener la nube de puntos resultante
cloud_filtered = passthrough.filter()

# Grabar el resultado en disco
