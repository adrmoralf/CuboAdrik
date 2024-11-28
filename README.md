# CuboAdrik
Proyecto sobre un Cubo de Rubik virtual a partir de un microcontrolador
## Descripción
Este proyecto consiste en la creación de un Cubo de Rubik virtual utilizando un microcontrolador. El objetivo es simular el funcionamiento de un Cubo de Rubik físico mediante software y hardware.

## Características
- Simulación del Cubo de Rubik en tiempo real
- Interfaz de usuario intuitiva
- Control mediante botones físicos y virtuales
- Código abierto y personalizable

## Requisitos
- Microcontrolador compatible. En mi caso utilicé [TM4C1294](https://www.ti.com/tool/MX/EK-TM4C1294XL) de la familia ARM Cortex M4F de Texas Instruments
- Conexión a una pantalla LCD. [VM800](https://www.mouser.es/ProductDetail/Bridgetek/VM800B50A-BK?qs=jtImr9hqsuEtyifgoc6xMQ%3D%3D&srsltid=AfmBOooI_CoYkO9euOHoyzpJSXLiM3ui12Z2mDLCmtUf60LfI_ZJ_99n)
- Acelerómetro
- Conexión a una fuente de alimentación

## Instalación
1. Clona el repositorio:
    ```bash
    git clone https://github.com/tu_usuario/CuboAdrik.git
    ```
2. Navega al directorio del proyecto:
    ```bash
    cd CuboAdrik
    ```
3. Instala [CodeComposer](https://www.ti.com/tool/CCSTUDIO#downloads) y crea un proyecto.
4. Copia las librerías y Rubik.c en tu proyecto.

## Uso
1. Conecta el microcontrolador a la pantalla LCD.
2. Carga el código en el microcontrolador.
3. Enciende el dispositivo y sigue las instrucciones en pantalla para comenzar a usar el Cubo de Rubik virtual.

## Contribuciones
Las contribuciones son bienvenidas. Por favor, sigue los pasos a continuación para contribuir:
1. Haz un fork del repositorio.
2. Crea una nueva rama con tu característica o corrección de errores:
    ```bash
    git checkout -b mi-nueva-rama
    ```
3. Realiza tus cambios y haz commit:
    ```bash
    git commit -m "Añadir nueva característica"
    ```
4. Sube tus cambios a tu repositorio fork:
    ```bash
    git push origin mi-nueva-rama
    ```
5. Abre un Pull Request en el repositorio original.

## Licencia
Este proyecto está licenciado bajo la Licencia MIT. Consulta el archivo `LICENSE` para más detalles.

## Contacto
Para cualquier consulta o sugerencia, por favor contacta a [adrmoralf@gmail.com](mailto:adrmoralf@gmail.com).