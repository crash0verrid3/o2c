<html>
<head>
	<title>Fancy CSS Demo</title>
	<meta charset="utf-8">
	<style type="text/css">
	<link href="https://fonts.googleapis.com/css?family=Arvo:400" rel="stylesheet">
	<link href="https://fonts.googleapis.com/css?family=Montserrat" rel="stylesheet">
	<style type="text/css">
.blank{}
.page_title{
	text-align: center;
	font-size: 24px;
}
.header_bar{
	text-align: center;
}
.top_bar{
	color: #ccc;
	font-family: 'Montserrat', sans-serif;
	border-bottom-style: groove;
	border-left-style: groove;
	border-right-style: groove;
	border-color: #333;
}
footer{
	color: #777;
	border-top-color: #333;
	border-top-style: groove;
	padding-top: 5pt;
	text-align: center;
	font-family: 'Arvo', sans-serif;
}
body{
	background-color: #111;
}
a{
	color: #777;
	text-decoration: none;
	font-family: 'Arvo', monospace;
}
a.sidebar_hrefs{
	font-size: 13pt;
	color: #777;
}
.body{
	font-family: 'Arvo', sans-serif;
	color: #888;
	position: relative;
	left: 10px;
	font-size: 13pt;
	width: 95%;
	align: center;
}
p{
	text-indent: 10px;
}
h3{
	text-align: center;
}
hr{
	border-color: #333;
}
	</style>
</head>
<body>
	<div class="top_bar">
		<div class="header_bar">
			<span class="page_title">
				&nbsp;CSS Template&nbsp;
			</span>
		</div>
		<hr style="width: 15%;">
		<br>
		<table style="text-align: center; width: 100%">
			<tr>
				<td>
					<a href="/link1.php" class="sidebar_href">Menu item 1</a>
				</td>
				<td>
					<a href="/link2.php" class="sidebar_href">Menu item 2</a>
				</td>
				<td>
					<a href="/link3.php" class="sidebar_href">Menu item 3</a>
				</td>
			</tr>
		</table>
		</div>
	</div>
	<div class="body">
		<br>
		<h3>
			Welcome to the Roaring Robotics Devlopment Page!
		</h3>
		<table>
			<tr style="">
				<b>
					<td>Date</td><td>Version</td><td>Build mode</td><td>What changed</td>
				</b>
			</tr>
			<tr>
				<td>7/23/2016</td><td>2016.1.1</td><td>Development</td><td>The robot now travels more straight due to some fine-tuning of the adjustment factor</td>
			</tr>
				<td>7/21/2016</td><td>2016.1</td><td>Development <i>untested</i></td><td>The scooper now holds in place when not moving by correcting for the effect of gravity (experimental)</td>
		</table>
*Releases can be downloaded at https://github.com/crash0verrid3/o2c/releases/*
	</div>
	<footer>
		Designed by Alex Anderson (2016).
	</footer>
</body>
</html>
