<html>
<body>
<?php
$serverName = "localhost";
$userName = "root";
$password = "";
$dbName = "oxiplus";

$conn = mysqli_connect($serverName, $userName, $password, $dbName);

if(!$conn){
	echo "Error: " . mysqli_connect_error();
	exit();
}

echo "Connection Success!<br><br>";

$id = isset( $_GET["Id"]);

$Fecha =  isset( $_GET["Fecha"]); 
$ESpO2 = $_GET["ESpO2"];

$query = $query = "UPDATE informacion SET Dato = '$ESpO2', Fecha = NOW() WHERE Id_dato = 1";
$result = mysqli_query($conn,$query);

echo "Insertion Success!<br>";
?>
</body>
</html>