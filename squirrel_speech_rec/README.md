<a id="top"/> 
# squirrel_speech_rec 

This folder holds files for perfoming speech recognition in ROS and python, as well as language specific database for command recongition. 

Based on 
'Zhang, A. (2015). Speech Recognition (Version 3.1) [Software]'
Available from <https://github.com/Uberi/speech_recognition#readme>

It is possible to use various online APIs to perform a speech recognition. The program records a audiofile and transmits it to the recongition service. the response is then used to generate commands for the robot. 

Maintainer: [**florianvoigtlaender**](https://github.com/**github-user**/) (**Florian Voigtländer**, **Festo Didactic SE**) - **florian.voigtlaender@festo.com**

##Contents

1. <a href="#1--installation-requirements">Installation Requirements</a>
2. <a href="#2--execution">Execution</a>
3. <a href="#3--software-architecture">Software architecture</a>
4. <a href="#4--troubleshooting">Troubleshooting</a>
5. <a href="#5--hardware">Hardware</a>
6. <a href="#6--languages">Languages</a>



## 1. Installation <a id="1--installation-requirements"/>
####Debian packages

`sudo apt-get install python-pyaudio python3-pyaudio`

For 64 bit system only:
`sudo apt-get install flac` 

For all:
`sudo apt-get install multimedia-jack`
select yes in the dialog window.
This can also be changed later with: 
`sudo dpkg-reconfigure -p high jackd2` and select "Yes" to do so.


## 2. Execution <a id="2--execution"/> 

`rosrun squirrel_speech_rec sq_ros_speech_rec <language> <device_ID>`
Starts a rosnode which listens to the microphone and sends requests to the google speech API. The result is then published to a topic.
The parameters are optional. `<language>` can be any language code which is supported by google for the voice recognition. For the parser, only German is supported currently. The `<device_ID>` completly depends on the used system and hardware. 

`rosrun squirrel_speech_rec sq_ros_speech_parser`
Starts a rosnode which checks the published message for known commands (for example: "Roboter gehe links"/"Robot go left"). If a command is recognized, a new message is published to a different topic.  

### Examples ###
`rosrun squirrel_speech_rec sq_ros_speech_rec` invokes the standard behaviour `rosrun squirrel_speech_rec sq_ros_speech_rec de 0`. 

`rosrun squirrel_speech_rec sq_ros_speech_rec de`

`rosrun squirrel_speech_rec sq_ros_speech_rec de-DE`

`rosrun squirrel_speech_rec sq_ros_speech_rec en 3`



### Testing the Speech Recognition for different Languages 
Similar to the usage of the rosnodes, a python script can be used to test the speech recongition on multiple OS without a ROS installation(so far MacOS and Ubuntu 14.04).

Start it by typing 
`python pure_py_speech_rec <language> <device_ID>`




### Record Audio 
Show available Audio devices:
* for recording
`aplay -l`
* for playing
`arecord -l`

Recoding a file in current directory named test.wav
`arecord -f S16_LE -r 16000 -d 5 -D hw:0,0 test.wav`

you may need to adjust the length of the recording (`-d` in seconds) and the audio device (`-D`).

Play the file
`aplay test.wav`


## 3. Software architecture <a id="3--software-architecture"/> 

**node name**: ![**squirrel_speech_rec**](https://github.com/squirrel-project//squirrel_recommender/master/software_architecture/sq_speech.PNG "Architecture")


## 4. Troubleshooting <a id="4--troubleshooting"/>

### "jack server is not running or cannot be started" or "Cannot lock down [...] byte memory area (Cannot allocate memory)" ###

Check if current user is in the ``audio`` group. 
Add your current user with ``sudo adduser $(whoami) audio``. 

reboot the system.

``pulseaudio --kill``
``jack_control start``




### Debugging audio devices ###
Test your speakers
`speaker-test -p 500 -D hw:1,0 -c 2`

Use audacity
`sudo apt-get install audacity`
You can generate various sinus and noise signals and change between devices and test them.

Trivial
* Check if headset/sound card is plugged in. 
* Check if headset is muted. (G930 shows a red LED at the tip of the microphone if it is muted)
* Check if headset is connected.
 


## 5. Hardware <a id="5--hardware"/>
Recommendation for use with robotino: Logitech G930 (wireless)
The robotino does not have a sound card, therefore a USB sound card is required to handle audio signals. The mentioned Headset has a built-in USB sound card and is tested to work with the robotino system. 
In general, the used audio device should support at least a rate of 16kHz.


 
## 6. Languages <a id="6--languages"/>
Languages supported by Google (excerpt):

* Dutch nl-NL
* English(UK) en-GB
* English(US) en-US
* Finnish fi
* French fr-FR
* German de-DE
* Italian it-IT
* Japanese ja
* Korean ko
* Mandarin Chinese zh-CN
* Norwegian no-NO
* Polish pl
* Portuguese pt-PT
* Russian ru
* Spanish(Spain) es-ES
* Swedish sv-SE
* Turkish tr

<a href="#top">top</a>
