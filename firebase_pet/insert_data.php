<?php

require_once 'firebase-php/src/firebaseLib.php';
date_default_timezone_set('Asia/Jakarta');

$url = 'https://pettracker-3a3ff.firebaseio.com/'; 
$token = 'mRWmz1qm1HvKakxxiGl1MW0EwySooyGgEG5dwXXl'; 
$node = '/Device1';

$credentials = 'pet123';

// check method 
if($_SERVER["REQUEST_METHOD"] == "GET" && !empty($_GET)){
	$api_key = $_GET["api_key"];

	// check api_key to credentials
	if($api_key == $credentials){
		$lat = $_GET["lat"];
		$lon = $_GET["lon"];
		$rssi = $_GET["rssi"];

		$lastUpdate = date("d-m-Y H:i:s");
		$new_data= array(
			'LastUpdate' => $lastUpdate,
			'Latitude' => $lat,
			'Longitude' => $lon,
			'RSSI' => $rssi
		);
		
		//print_r($new_data);
		 
		$firebase = new \Firebase\FirebaseLib($url, $token);
		$firebase->update($node, $new_data);
		print("Update to firebase success...");
	} else {
		print("Wrong credentials, check API key...");
	}
} else {
	print("Method doesn't support");
}

?>