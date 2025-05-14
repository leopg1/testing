#!/bin/bash

echo "=== Script de rulare Robot Labirint pe Raspberry Pi ==="
echo "Acest script gestionează migrarea și rularea proiectului în Docker pe Raspberry Pi"

# Verifică dacă Docker este instalat
if ! command -v docker &> /dev/null; then
    echo "EROARE: Docker nu este instalat. Instalează Docker prima dată."
    echo "Poți folosi: curl -sSL https://get.docker.com | sh"
    exit 1
fi

# Verifică dacă docker-compose este instalat
if ! command -v docker-compose &> /dev/null; then
    echo "docker-compose nu este instalat. Se instalează..."
    sudo apt install -y docker-compose
fi

# Creează directoarele pentru date persistente
mkdir -p logs maps

# Construiește imaginea Docker
echo "Construiește imaginea Docker..."
docker-compose build

# Rulează containerul
echo "Pornire aplicație în container Docker..."
docker-compose up

# Instrucțiuni pentru după oprire
echo ""
echo "Aplicația s-a oprit. Pentru a o reporni, rulează: docker-compose up"
echo "Pentru a intra în container pentru debugging: docker-compose run --entrypoint bash maze_robot"
