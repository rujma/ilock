CREATE TABLE IF NOT EXISTS RFID (
	idRFID TEXT PRIMARY KEY,
	nameRFID TEXT,
	admin BOOLEAN DEFAULT 0
);

CREATE TABLE IF NOT EXISTS Face (
	faceID INTEGER PRIMARY KEY AUTOINCREMENT,
	idRFID TEXT,
	FOREIGN KEY (idRFID) references RFID(idRFID) on update cascade on delete set null
);

CREATE TABLE IF NOT EXISTS Image (
	imageID INTEGER PRIMARY KEY AUTOINCREMENT,
	faceID INTEGER NOT NULL,
	facePATH TEXT,
	foreign key (faceID) references Face(faceID) on update cascade on delete cascade
);