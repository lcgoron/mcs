# Force 256 color gnome-terminal for vim. #
if [[ TERM=="xterm" && COLORTERM==gnome* ]]; then
  export TERM="xterm-256color"
fi
