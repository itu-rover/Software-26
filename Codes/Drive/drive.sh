!/bin/bash
echo "One Shell Script to Conquer All Scripts"

# Script'in bulunduğu dizini al
DIR=$(dirname "$(realpath "$0")")

PIDS=$(lsof -t -i :6102)

if [ -n "$PIDS" ]; then
    echo "⚠️ 6102 portunu kullanan process(ler) bulundu: $PIDS"
    kill -9 $PIDS
    echo "✅ Hepsi kapatıldı."
fi

IP=$(ip -4 addr | grep -oP '(?<=inet\s)192\.168\.1\.\d+' | head -n 1)

if [ -z "$IP" ]; then
    echo "❌ IP adresi bulunamadı! Kablo takılı mı? wifi bağlandın mı?"
    exit 1
fi

echo "✅ Bilgisayar IP adresi bulundu: $IP"

# Compile with all source files and include directory
gcc -o3 -I"$DIR/include" "$DIR/src/udp_server.c" "$DIR/src/joystick_handler.c" "$DIR/src/network.c" -o "$DIR/udp_server" -lpthread -DSERVER_IP="\"$IP\""

if [ $? -eq 0 ]; then
    echo "✅ Derleme başarılı."
else
    echo "❌ Derleme hatası!"
    exit 1
fi

echo "🚀 Program başlatılıyor..."
"$DIR/udp_server"
