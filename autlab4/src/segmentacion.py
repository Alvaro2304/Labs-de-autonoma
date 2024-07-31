# -*- coding: utf-8 -*-


# Cargar la nube de puntos


# Crear el objeto para la segmentación
seg = cloud.make_segmenter()

# Asignar el modelo que se desea ajustar
seg.set_model_type(pcl.SACMODEL_PLANE)
# Uso de RANSAC
seg.set_method_type(pcl.SAC_RANSAC)

# Máxima distancia
max_distance = 1
seg.set_distance_threshold(max_distance)
# Función de segmentación con RANSAC para obtener los índices de los inliers
inliers, coefficients = seg.segment()

# Extracción de  inliers
cloud_inliers = cloud.extract(inliers, negative=False)

# Grabar el resultado en disco
