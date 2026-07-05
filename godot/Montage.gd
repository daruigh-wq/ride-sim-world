extends Node
func _ready() -> void:
	var degs := [0,45,90,135,180,225,270,315]
	var cols := 4; var rows := 2
	var cw := 460; var ch := 320
	var grid := Image.create(cols*cw, rows*ch, false, Image.FORMAT_RGBA8)
	grid.fill(Color(1,1,1,1))
	for i in degs.size():
		var img := Image.load_from_file("res://../bike_side_ik_%d.png" % degs[i])
		var src := img.get_region(Rect2i(330, 380, 980, 700))
		src.resize(cw, ch, Image.INTERPOLATE_BILINEAR)
		src.convert(Image.FORMAT_RGBA8)
		grid.blit_rect(src, Rect2i(0,0,cw,ch), Vector2i((i%cols)*cw, (i/cols)*ch))
	grid.save_png("res://../ik_montage.png")
	get_tree().quit()
