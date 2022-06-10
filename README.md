# Actividad-3-SCII
**IMPORTANTE!!**
Códigos correspondientes al desarrollo del informe 3 de la materia Sistemas de control II, control por realimentación de estados En este archivo se detalla la funcionalidad de cada código como orientación. Tomar como finales los códigos que se mencionan AQUÍ, el resto son iteraciones hasta lograr todo lo requerido y pueden no funcionar bien.

***CÓDIGOS FINALES***

MOTOR1_INTEGRADOR_OBSERVADOR_ZONA_MUERTA: Contempla el control por realimentación de estados de un motor de referencia y torque de perturbación variables con un observador cuando no puede medirse la corriente. También se agrega una alinealidad que puede ser modificada para evaluar el comportamiento del sistema

AVION1_INTEGRADOR: Muestra el controlador propuesto para rangos de validez del modelo del cabeceo del avión

AVION1_INTEGRADOR_OBSERVADOR_2_VAR: Agrego un observador cuando se pueden medir alpha y h

AVION1_INTEGRADOR_OBSERVADOR_ZONA_MUERTA: Agrego zona muerta al código anterior, la zona muerta en el avión no puede ser muy grande

PENDULO1_OBSERVADOR_SISTNL_ZONA_MUERTA: Control de una grúa con cambio de referencia y masa trasladada, presenta un observador ya que no solo pueden medirse el desplazamiento y ángulo, y una zona muerta en el actuador

***ITERACIONES***

A continuación se presentan todos los códigos utilizados para dar finalmente con los mejores casos de los códigos finales

MOTOR1_INTEGRADOR: Planteo del control discreto por realimentación de estados del motor con referencia y perturbación variable

MOTOR1_INTEGRADOR_ZONA_MUERTA: Planteo del control discreto por realimentación de estados del motor con referencia y perturbación variable, agregando zona muerta como 
alinealidad

MOTOR1_INTEGRADOR_OBSERVADOR: Planteo del control discreto por realimentación de estados del motor con referencia y perturbación variable, cuando hay variables que no pueden medirse y deben estimarse

MOTOR1_INTEGRADOR_OBSERVADOR_ZONA_MUERTA: Planteo del control discreto por realimentación de estados del motor con referencia y perturbación variable, cuando hay variables que no pueden medirse y deben estimarse, y además incorporando una zona muerta como alinealidad 

AVION1_INTEGRADOR: Planteo del control discreto por realimentación de estados del avion sin superar los límites del modelo lineal

AVION1_INTEGRADOR_ZONA_MUERTA: Planteo del control discreto por realimentación de estados del avion, agregando zona muerta como alinealidad

AVION1_INTEGRADOR_OBSERVADOR: Planteo del control discreto por realimentación de estados del avion, cuando hay variables que no pueden medirse y deben estimarse con un observador

PENDULO1_INTEGRADOR: Planteo del control discreto por realimentación de estados del pendulo en el equilibrio estable, con variación paramétrica de la masa y cambio de referencia. No se logran buenos resultados, se busca la solución implementando controlador variable

PENDULO1_INTEGRADOR_DOBLE_K: Planteo del control discreto por realimentación de estados del pendulo en el equilibrio estable, con variación paramétrica de la masa y cambio de referencia, cambiando de controlador cuando es necesario debido al cambio de masa

PENDULO1_INTEGRADOR_DOBLE_K_DOBLE_OBSERVADOR: Planteo del control discreto por realimentación de estados del pendulo en el equilibrio estable, con variación paramétrica de la masa y cambio de referencia, cambiando de controlador cuando es necesario debido al cambio de masa, y a su vez, cambiando de observador cuando es necesario para la estimación de estados

PENDULO1_INTEGRADOR_DOBLE_K_DOBLE_OBSERVADOR_SIST_NL: Planteo del control discreto por realimentación de estados del pendulo en el equilibrio estable, con variación paramétrica de la masa y cambio de referencia, cambiando de controlador cuando es necesario debido al cambio de masa, y a su vez, cambiando de observador cuando es necesario para la estimación de estados. Todo aplicado sobre el sistema no lineal.

En la mayoría de los códigos está dispuesta una sección que puede descomentarse para agregar alinealidad al actuador, en todos los casos zonas muertas
