# Self-Balancing-Robot-

--FRANCAIS--

Ce projet à été réalisé individuellement en parallèle de mes études, reprend un projet que nous avions réalisé en ING1 à l'ECE Paris. Mais cette fois, au lieu de tout avoir déjà donné et pensé dans un kit, je me suis lancé le défi de tout faire moi même, de A à Z, de la 3D à la création du PCB, au codage du PID. Voici le résultat final : 

--ENGLISH

I carried out this project independently alongside my studies, revisiting a project we had originally completed during our first year of engineering school (ING1) at ECE Paris. This time, however, instead of using a kit where everything was pre-designed and provided, I challenged myself to build the entire thing from scratch—from 3D modeling and PCB design to programming the PID controller. Here is the final result : 

<p align="center">
  <img width="393" height="606" alt="Self_Balancing" src="https://github.com/user-attachments/assets/a890c99d-8c80-4814-a75c-9c0a80573a46" />
</p>

<p align="center">
  <img width="393" src="media/self_balancing.webp" alt="Démonstration du robot" />
</p>

--FRANCAIS--
Si souhaité, ce robort peut-être transformé en robot télécommandé, il suffit simplement de créer un serveur web sur l'EPS32 et d'y ajouter une modification de la consigne d'angle, ou de set un pitch/roll cible. 

Les difficultés rencontrés ont été sur le codage du PID, le fait que le buck-down choisi, le Mini360, ne donnait probablement pas suffisament de courant, bridant le couple et compliquant ainsi le codage du PID, j'ai également eu quelque problèmes avec le placement de mon MPU-6050, qui est un module bassse qualité, qui m'a beaucoup ralenti, en effet, pas malin d'acheter le coeur du projet à 3e sur Aliexpress au lieu de 15e chez le constructeur. Egalement celui-ci à été mal placé, ce que j'ai corrigé avec le point de col sur le coté pour le mettre à l'horizontal et non à la verticale ccomme il l'était. Aussi, j'ai "verser du cuivre" en GND sous l'antenne, ce qui n'est pas bon pour l'intégrité du signal radio et réduisait ainsi mes performances WIFI.

--ENGLISH--
If desired, this robot can be converted into a remote-controlled unit; this simply requires setting up a web server on the ESP32 and adding a way to modify the angle setpoint or define a target pitch/roll.

I encountered difficulties with the PID coding, partly because the chosen step-down converter (the Mini360) likely didn't supply enough current—limiting torque and complicating PID tuning. I also faced issues with the placement of the MPU-6050; it was a low-quality module that really slowed me down—in hindsight, buying the project's core component for €3 on AliExpress instead of €15 from the manufacturer wasn't the smartest move. Furthermore, its initial positioning was incorrect; I fixed this by mounting it horizontally (using a side-mounted glue point) rather than vertically as it had been before. Finally, I had "poured copper" (a ground plane) beneath the antenna, which compromised radio signal integrity and reduced Wi-Fi performance.
