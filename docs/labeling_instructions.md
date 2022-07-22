# Labeling team instructions


### COMPUTER SETUP:


If you have NOT labeled yet, install VoTT for your operating system: https://github.com/Microsoft/VoTT/releases

(Go to the "releases" area on the right and download it for your operating system under assets) 

Windows: win32.exe

Linux: linux.snap (You will need snap)

Mac: darwin.dmg


You will also need a way to zip and unzip folders into a .zip file. If you are on Windows, I recommend 7zip.



### PROJECT SETUP:


1. Download images in a .zip folder from the READY_FOR_LABELING folder pinnged in Discord in #team_members. You will need to unzip these files to a known location; I recommend the desktop. Unzip the files to a folder of the SAME NAME as the .zip file (without the .zip), this is so we can keep track internally of what has been labeled. They contain a number of images that need to be sorted that are, at most, 500. This will be the `label folder` which will be reffered to later in the instructions.

2. Once VoTT has been installed, launch VoTT. 

3. Click on the `+` sign for a new project. 

- Enter the text `Scion` into the `Display Name` field.

- In `Source Connection`, click on `Add Connection`. From here, Enter the text `labels` into the `Display Name` field. Click on the `Select Provider` field, then click on `Local file system`. For the Folder Path field, we are going to select the `label folder` we unzipped in part 1. After selecting the unzipped folder, click on `Save Connection`. You should return to the initial page and see a green tab come up in VoTT saying `Successfully saved labels`.

- Click on `Source Connection` and select the `labels` with the `label folder`.

- Click on `Target Connection` and select the `labels` with the `label folder`.

- At the very bottom, click `Save Project`.

#### At this point, you should be seeing images loaded into your VoTT. If you do not, ASK IAN OR SOMEONE ELSE for help.

#### The next step is VERY IMPORTANT! If the export settings are incorrect, your labels will not be able to be parsed and will be DISCARDED.

- Click on the arrow to the left (fourth picture down) to open the Export Settings.

- Click on the `Provider` field and change it from `VoTT JSON` to `Comma Separated Values (CSV)`. 

- Click on the `Include Images` field to UNCHECK it. We do not need a second export of image data.

- Click on `Save Export Settings` to save your export settings.


### LABELING:

1. On the right, you will see an area called `tags`. Below in this document is a reference for what label belongs to what for each competition object. Your job is to use the `Draw Rectangle` tool in the top of the VoTT window (Hotkey R). If your `tags` section is empty, begin by adding `tags` for what you are labeling. Scroll down in this document for what labels to make for each of your images. 

- To add a new tag, click on the `+` sign.

- Type in the name of the tag. These names are provided for you in this document below, depending on what you are labeling. Type these in EXACTLY VERBATIM, caps and spaces and underscores MATTER. If you are unsure about the label names, ask Ian or someone else for help.

- Draw a rectangle around the relevant areas in the image, then type the key on your keyboard associated with the label.  These CAN overlap when relevant.

![image](/docs/img/ex_1_labels.png)

(Here the label for the `rgate` is 2, so I select the area of the `rgate` and press 2 on my keyboard.)

- Once you are done with all labels for the image, hold `CTRL` and then `S` on your keyboard to save, then use the `Down` arrow key to move on to the next image.

- Repeat for all images.


### UPLOADING LABELS:

1. Once the images are complete, Hit the `Export` button to the right of the `Save Icon` on the top bar in Vott. (Hotkey `CTRL`+ `E`). This will export a .csv of everything you have labeled.

2. Use a program such as 7zip to zip the project folder. This is so it can be sent over the internet.

![image](/docs/img/zip_ex.png)

3. RENAME this .zip folder from `[TO_LABEL]` to `[LABELED]` at the beginning. 

4. Upload it to the second link pinned in #team_members in Discord. 


### LABEL GUIDE:


![image](/docs/img/Full%20Mob.png)

LABEL: `Full Mob`

The Full Mob label is for the white background and gangser itself. Any partial image of these shall be selected.

![image](/docs/img/Mob%20Circle.png)

LABEL: `Mob Circle`

The Circle Mob label is for the circle in the upper left. Only label it if it is the FULL circle.

![image](/docs/img/Mob%20Trap.png)

LABEL: `Mob Trap`

The Mob Trap label is for the Trapezoid on the bottom right. Only label it if it is the FULL Trapezoid.

![image](/docs/img/lgate.png)

LABEL: `lgate`

The lgate label is for the left half of the competition gate; the one with the gman. This can, and will, overlap with the rgate label in the center pole, as both the lgate and rgate will have a center pole. Label the lgate for left half of the gate in partial or full view.

![image](/docs/img/rgate.png)

LABEL: `rgate`

The rgate label is for the right half of the competition gate; the one with the gangster. This can, and will, overlap with the lgate label in the center pole, as both the lgate and rgate will have a center pole. Label the rgate for right half of the gate in partial or full view.

![image](/docs/img/oct.png)

LABEL: `oct`

The oct label is for the white octagon pvc floating on top of the pool. Label the octagon with any part of it in view.

![image](/docs/img/bottle.png)

LABEL: `bottle`

The bottle label is for the purple bottle sitting at the bottom of the pool. Label the bottle with any part of it in view.

![image](/docs/img/guide.png)

LABEL: `guide`

The guide label is for the orange guide that indicates direction to the robot. Label the guide with any part of it in view.

![image](/docs/img/smash_axe.png)

LABEL: `smash_axe`

The smash axe label is for the axe on the orange sign. Label any part of the AXE ITSELF, NOT the larger orange box.

#####![image](/docs/img/cash_grab.png)
(The cash grab was not done in time)

![image](/docs/img/alch_bottle.png)

LABEL: `alch_bottle`

The alch bottle is for the bottle of booze in the smash and grab section. Any part of this bottle should be labeled.

![image](/docs/img/alch_barrel.png)

LABEL: `alch_barrel`

The alch barrel is for the booze barrel image in the smash and grab section. Any part of this barrel should be labeled.

![image](/docs/img/alch_container.png)

LABEL: `alch_container`

The alch container label is the larger white container that holds the bottle and barrel labels. Any part of this container should be labeled, as long as it has either an alch_barrel or alch_bottle partial image.

![image](/docs/img/alch_lid.png)

LABEL: `alch_lid`

The alch lid label is the orange lid with the blue PVC in the smash and grab section. It must be the FULL lid to label it.
