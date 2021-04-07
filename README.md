# Automated-Warehouse-Management-System
Automation of warehouse management system using ROS and GAZEBO along with IOT and Google app scripting.

## <h3 style="background:powderblue;" align="left">EYRC - Team Details</h3>
* Team ID : eYRC-VB#0302
* Theme : Vargi Bots

## <h3 style="background: Lavender;" align="left">About Theme - Vargi Bot</h3>

<p>Inspired by this visualisation of Industry 4.0, the current edition of the e-Yantra Robotics Competition features a theme called ‘Vargi-Bots’. Vargi is taken from a Sanskrit word, Vargikaran (वर्गीकरण) which means to separate objects based on their category. The theme is set in the abstraction of a warehouse management system designed in Gazebo, which is a 3D dynamic simulator used to efficiently simulate robots in complex environments.</p>

<p>The arena is an automated warehouse setting where essential packages are required to be sent out to different parts of a city. Since Industry 4.0 heavily focuses on automation here the warehouse will only consist of two industrial robotic arms which will be used by the teams. As the requirements are sent to the warehouse, one robotic arm will identify the packages from a shelf and place them on a conveyor belt and the other robotic arm at the end of the conveyor belt will pick these objects from the conveyor and place them into bins. Each bin represents a destination for the package. As the packages are sent out from the warehouse there will also be alerts sent to the user via email notifying them about the package being shipped from the warehouse.</p>

<p>The packages to be delivered have their own priorities. Packages having a higher priority are intended for a natural disaster or a pandemic situation. Other packages with lower priorities are for general purposes. Similar to a conductor in an orchestra, in this theme, the participants have to design their own conductor (controller) for their warehouse to make smart decisions in order to deliver high priority packages as quickly as possible.</p>

## <h3 align="left">Introducton</h3>

<b><u>Overview and stages:</u></b> <p>Vargi Bot being the allttoed theme, we realized the problem statement given every week using ROS(Robot Operating System), GAZEBO (Open-source 3D robotics simulator) and Google App Scripting. We controlled robotic arm in Gazebo using Python scripts and Rviz planner.</p>

<b><u>Project flow:</u></b> <p>Initially the packages in the shelf are detected using 2D Camera and the captured image feed is processed to detect the QR Code on each of the packages. Denoising of the image is done to get better quality image and to detect all 12 packages. Then the Inventory sheet of INVENTORY MANAGEMENT SYSTEM spreadsheet is updated. As and when the incoming orders are received, the IncomingOrdrs spredsheet is updated.The ur5_1 arm then picks packages according to incoming orders and the packages are placed on conveyor belt without colloiding with its environment. As soon as the package is placed on the conveyor belt, the OrdersDispatched spredshhet is updated and an alert mail is sent saying <b>Order Dispatched</b> with the order details. The conveyor belt carries the package to the ur5_2 arm which picks it and drops it in corresponding bins. As soon as the package is dropped into the bin, OrdersShipped spredsheet is updated and an alert mail is sent saying <b>Order Shipped</b> with the order details. The Dashboard spredsheet is updated time to time taking values from the other sheets.  </br></br> Dashboard webpage is created, wherein all the details with repect to orders are updated referencing from Dashboard spredsheet of INVENTORY MANAGEMENT SYSTEM. </br>The location of orders are pointed on map using pointers of different colors:</p>
 
<b>Red pointer :</b> Neither Dispatched nor shipped</br>
<b>Yellow pointer :</b> Dispatched but not yet shipped</br>
<b>Green pointer :</b> Dispatched and shipped successfully</br>

<p>The time taken to ship the orders (Time duration between Order time and Shiping time) is indicated using bar graph. Bar graph is indicated with Red, Yellow and Green for Medicine, Food and Clothes repectively.</p>



## <h3 style="background: Lavender" align="left">Implementation Video</h3>

[Click here](https://www.youtube.com/watch?v=6J8YmJlxjRo) to view the YouTube video.
