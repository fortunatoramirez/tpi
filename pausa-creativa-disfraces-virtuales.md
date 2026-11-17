# Pausa creativa de visión por computadora — Disfraces virtuales en tiempo real

**Técnicas y Procesos de Imágenes · Actividad en parejas · 40 minutos aproximadamente**

Hoy la cámara cambia de escenario. Hagan una pausa en el proyecto de la banda y conviertan la computadora en una cabina de disfraces. Su misión es inventar un personaje que otra pareja pueda reconocer con solo mirarse en la pantalla. Una persona lleva el teclado y la otra modela. A mitad de la actividad, intercambien lugares.

## 1. Abran la cabina · 5 minutos

Necesitan su computadora, Python y la cámara que ya utilizaron en la práctica 8. Si usan el iPhone, conéctenlo como de costumbre, colóquenlo en horizontal y cierren cualquier programa que esté ocupando su cámara.

Creen un archivo llamado `cabina.py` y copien el programa base. En `CAMARA` coloquen el mismo índice o dirección de video que les funcionó en la práctica 8. Ejecuten `python cabina.py` desde la terminal de VS Code. Si Python indica que falta `cv2` o `numpy`, ejecuten `python -m pip install opencv-python numpy` con ese mismo intérprete.

```python
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


CAMARA = 0


def dibujar_disfraz(imagen, rostro):
    x, y, w, h = rostro

    # Las posiciones se miden desde la esquina superior izquierda del rostro.
    izquierdo = (x + int(0.30 * w), y + int(0.42 * h))
    derecho = (x + int(0.70 * w), y + int(0.42 * h))
    radio = max(2, int(0.16 * w))
    grosor = max(2, round(0.025 * w))
    color = (0, 210, 255)  # OpenCV usa azul, verde, rojo (BGR).

    cv2.line(imagen, (izquierdo[0] + radio, izquierdo[1]),
             (derecho[0] - radio, derecho[1]), color, grosor)
    for centro in (izquierdo, derecho):
        cv2.circle(imagen, centro, radio, color, grosor, cv2.LINE_AA)

    # Agreguen aqui el detalle que convierte los lentes en su personaje.


def crear_mascara(forma, rostros):
    mascara = np.zeros(forma, dtype=np.uint8)
    for x, y, w, h in rostros:
        centro = (x + w // 2, y + h // 2)
        ejes = (int(0.44 * w), int(0.50 * h))
        cv2.ellipse(mascara, centro, ejes, 0, 0, 360, 255, -1)
    return mascara


def main():
    ruta = Path(cv2.data.haarcascades) / "haarcascade_frontalface_default.xml"
    detector = cv2.CascadeClassifier(str(ruta))
    if detector.empty():
        raise SystemExit("No se pudo cargar el detector frontal de OpenCV.")

    camara = cv2.VideoCapture(CAMARA)
    modo = 1
    aviso = ""
    nombres = {1: "Encuadre", 2: "Disfraz", 3: "Mascara"}
    try:
        if not camara.isOpened():
            raise SystemExit("Revisa CAMARA y cierra otras apps que usen la camara.")
        while True:
            ok, original = camara.read()
            if not ok:
                print("Sin imagen. Revisa la conexion y vuelve a ejecutar.")
                break
            alto, ancho = original.shape[:2]
            escala = min(800 / ancho, 600 / alto, 1.0)
            espejo = cv2.flip(cv2.resize(original, None, fx=escala, fy=escala), 1)
            gris = cv2.cvtColor(espejo, cv2.COLOR_BGR2GRAY)
            rostros = detector.detectMultiScale(gris, scaleFactor=1.1,
                                                minNeighbors=5, minSize=(40, 40))
            salida = espejo.copy()
            if modo == 3:
                mascara = crear_mascara(gris.shape, rostros)
                salida = cv2.cvtColor(gris, cv2.COLOR_GRAY2BGR)
                cv2.copyTo(espejo, mascara, salida)

            for x, y, w, h in rostros:
                if modo == 1:
                    cv2.rectangle(salida, (x, y), (x+w, y+h), (0, 230, 0), 2)
                else:
                    dibujar_disfraz(salida, (x, y, w, h))

            # En modo 3, comparen original, mascara y resultado de izquierda a derecha.
            vista = salida
            if modo == 3:
                paneles = [espejo, cv2.cvtColor(mascara, cv2.COLOR_GRAY2BGR), salida]
                vista = cv2.hconcat([cv2.resize(p, None, fx=0.5, fy=0.5) for p in paneles])
            estado = nombres[modo] if len(rostros) else "Mira de frente y busca luz uniforme"
            pantalla = cv2.copyMakeBorder(vista, 58, 0, 0, 0, cv2.BORDER_CONSTANT)
            cv2.putText(pantalla, "1 Encuadre | 2 Disfraz | 3 Mascara | Espacio Foto | Q Salir",
                        (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
            cv2.putText(pantalla, f"{estado}   {aviso}", (6, 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (70, 230, 255), 1)
            cv2.imshow("Cabina", pantalla)
            tecla = cv2.waitKey(1) & 0xFF
            if tecla in (ord("q"), ord("Q"), 27):
                break
            if tecla in (ord("1"), ord("2"), ord("3")):
                modo = int(chr(tecla))
                aviso = ""
            if tecla == 32:
                nombre = datetime.now().strftime("disfraz_%H%M%S_%f.jpg")
                aviso = f"Foto: {nombre}" if cv2.imwrite(nombre, salida) else "Error al guardar"
    finally:
        camara.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
```

## 2. Prueben el espejo y los lentes · 5 minutos

Hagan clic en la ventana **Cabina** para que reciba las teclas. Presionen **1** y miren de frente hasta ver el recuadro verde. Muévanse hacia un lado. Después acérquense despacio y observen cómo cambia el ancho del recuadro.

Presionen **2** para ponerse los lentes. El detector entrega la esquina `(x, y)`, el ancho `w` y el alto `h` del rostro. El programa estima dónde dibujar los ojos a partir de ese rectángulo. **Prueba rápida:** si los lentes quedan en las cejas, ¿qué número cambiarían para bajarlos? Busquen `0.42 * h` en ambos centros. Aumenten un poco la fracción, guarden y vuelvan a ejecutar para comprobar su predicción.

## 3. Inventen un personaje · 15 minutos

Intercambien teclado y cámara. Elijan en secreto un personaje y modifiquen **solo la función `dibujar_disfraz`**. Agreguen un rasgo que permita reconocerlo aun si quien modela se acerca o se mueve. Usen fracciones de `w` y `h` para colocar sus dibujos.

¿Necesitan una pista para empezar? Para un robot, prueben una antena. Peguen estas líneas dentro de la función, en el lugar del comentario final. Para un gato, prueben tres líneas de bigote a cada lado. Para un pirata, conviertan uno de los lentes en un parche con un círculo relleno.

```python
    base = (x + w // 2, y + int(0.12 * h))
    punta = (base[0], y - int(0.15 * h))
    cv2.line(imagen, base, punta, color, grosor)
    cv2.circle(imagen, punta, max(3, int(0.06 * w)), (0, 0, 255), -1)
```

Dejen un poco de espacio sobre la cabeza para ver la antena. Al probar su diseño, pidan a su pareja que se aleje un paso. ¿El accesorio sigue colocado donde querían?

## 4. Descubran qué hace la máscara · 5 minutos

Presionen **3**. De izquierda a derecha verán **el espejo original**, **la máscara** y **el resultado con el disfraz**. Levanten una pluma de color junto a la mejilla y muévanla hasta que entre en el óvalo. Observen en qué momento recupera su color en la imagen de la derecha.

En el panel central, los píxeles blancos indican dónde se copia el color original sobre la imagen gris. El óvalo sigue al recuadro facial y selecciona todo lo que quede dentro, incluso la pluma. Para experimentar, cambien `0.44 * w` en `crear_mascara`: antes de ejecutar, predigan qué pasará con el ancho de la zona a color.

## 5. Jueguen «Adivina el disfraz» · 10 minutos

Inviten a otra pareja a su estación. Una persona se coloca frente a la cámara, se pone el disfraz con **2** y tiene diez segundos para adivinar el personaje. Puede pedir una pista haciendo una pose. Luego cambien de estación.

**Para cerrar**, muestren el disfraz en movimiento y expliquen qué valor ajustaron para que siguiera al rostro. La demostración en vivo es suficiente. Si quienes aparecen quieren llevarse una foto, presionen **Espacio**. Se guarda el resultado en la carpeta desde la que ejecutaron el programa.

| Si ocurre esto | Prueben esto |
|---|---|
| Aparece otra cámara | Copien en `CAMARA` el índice o la dirección que ya probaron con el iPhone. |
| El recuadro aparece y desaparece | Miren de frente, acerquen un poco el rostro y busquen luz de frente. Este detector funciona mejor con la cara frontal. |
| Las teclas no responden | Hagan clic dentro de **Cabina** y presionen la tecla sin usar Enter. |
| Cambiaron el dibujo y sigue igual | Cierren con **Q**, guarden el archivo y ejecútenlo otra vez. |
