import omni.replicator.core as rep
# 指定相机在场景中的活动范围
sequential_pos = [(-1.3, 0.5, 0.7), (-0.8, 0.5, 1.1)]
# 指定相机的对焦点，即指定它拍摄某一空间点的图像
look_at_postion = (-1.1, 2.20, 1)
with rep.new_layer():
    CKOOL_Y = 'omniverse://localhost/model/object/snack/ckool_yellow.usd'
    CKOOL_R = 'omniverse://localhost/model/object/snack/ckool_red.usd'
    CKOOL_P = 'omniverse://localhost/model/object/snack/ckool_purple.usd'
    CHENPIDAN_R = 'omniverse://localhost/model/object/snack/chenpidan_red.usd'
    CHENPIDAN_Y = 'omniverse://localhost/model/object/snack/chenpidan_yellow.usd'
    GUODONG_O = 'omniverse://localhost/model/object/snack/guodong_orange.usd'
    GUODONG_P = 'omniverse://localhost/model/object/snack/guodong_purple.usd'
    YIDA_O = 'omniverse://localhost/model/object/snack/yida_orange.usd'
    YIDA_R = 'omniverse://localhost/model/object/snack/yida_red.usd'
    YILIDUO = 'omniverse://localhost/model/object/snack/yiliduo.usd'


    # create randomizer function conference table assets.
    # This randomization includes placement and rotation of the assets on the surface.
    def env_ckool_y(size=5):
        ckool_y = rep.randomizer.instantiate(
            rep.utils.get_usd_files(CKOOL_Y, recursive=False),
            size=size,
            mode="scene_instance",
        )
        rep.modify.semantics([("class", "ckool")])
        with ckool_y:
            rep.modify.pose(
                position=rep.distribution.uniform((-1.3, 2.2, 0.9), (-1.1, 2.2, 1.1)),
                # position=(-0.91, 2.18, 0.96),
                # position=rep.distribution.uniform((-1.9, 2.16, 0.6), (-1.9, 2.2, 0.8)),
                rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 0)),
                scale=rep.distribution.uniform(0.03, 0.04)
            )
        return ckool_y.node
    rep.randomizer.register(env_ckool_y)

    def env_ckool_r(size=5):
        ckool_r = rep.randomizer.instantiate(
            rep.utils.get_usd_files(CKOOL_R, recursive=False),
            size=size,
            mode="scene_instance",
        )
        rep.modify.semantics([("class", "ckool")])
        with ckool_r:
            rep.modify.pose(
                # position=(-1.91, 2.18, 0.96),
                position=rep.distribution.uniform((-1.7, 2.16, 0.9), (-1.5, 2.23, 1.1)),
                rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 0)),
                scale=rep.distribution.uniform(0.03, 0.04)
            )
        return ckool_r.node
    rep.randomizer.register(env_ckool_r)

    def env_ckool_p(size=5):
        ckool_p = rep.randomizer.instantiate(
            rep.utils.get_usd_files(CKOOL_P, recursive=False),
            size=size,
            mode="scene_instance",
        )
        rep.modify.semantics([("class", "ckool")])
        with ckool_p:
            rep.modify.pose(
                position=rep.distribution.uniform((-1.3, 2.2, 0.9), (-1.1, 2.2, 1.1)),
                rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 0)),
                scale=rep.distribution.uniform(0.03, 0.04)
            )
        return ckool_p.node
    rep.randomizer.register(env_ckool_p)

    def env_chenpidan_r(size=5):
        chenpidan_r = rep.randomizer.instantiate(
            rep.utils.get_usd_files(CHENPIDAN_R, recursive=False),
            size=size,
            mode="scene_instance",
        )
        rep.modify.semantics([("class", "chenpidan")])
        with chenpidan_r:
            rep.modify.pose(
                position=rep.distribution.uniform((-1.7, 2.16, 0.9), (-1.5, 2.23, 1.1)),
                rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 0)),
                scale=rep.distribution.uniform(0.03, 0.04)
            )
        return chenpidan_r.node
    rep.randomizer.register(env_chenpidan_r)


    def env_chenpidan_y(size=5):
        chenpidan_y = rep.randomizer.instantiate(
            rep.utils.get_usd_files(CHENPIDAN_Y, recursive=False),
            size=size,
            mode="scene_instance",
        )
        rep.modify.semantics([("class", "chenpidan")])
        with chenpidan_y:
            rep.modify.pose(
                position=rep.distribution.uniform((-1.5, 2.16, 0.8), (-1.3, 2.23, 1.3)),
                rotation=rep.distribution.uniform((0, 0, 0), (0, 360, 0)),
                scale=rep.distribution.uniform(0.04, 0.05)
            )
        return chenpidan_y.node


    rep.randomizer.register(env_chenpidan_y)

    # Setup camera and attach it to render product

    # sphere lights for bonus challenge randomization
    def sphere_lights(num):
        lights = rep.create.light(
            light_type="Sphere",
            temperature=rep.distribution.normal(6500, 500),
            intensity=rep.distribution.normal(35000, 5000),
            position=rep.distribution.uniform((-300, -300, -300), (300, 300, 300)),
            scale=rep.distribution.uniform(50, 100),
            count=num,
        )
        return lights.node


    rep.randomizer.register(sphere_lights)


    # 检测目标随机位置生成
    def get_shapes():
        shapes = rep.get.prims(semantics=[('class', 'ckool'),
                                          ('class', 'chenpidan'), ('class', 'guodong'), ('class', 'yida'),
                                          ('class', 'yiliduo')])
        with shapes:
            rep.modify.pose(
                position=rep.distribution.uniform((-0.9, 2.29, 0.99), (-1.2, 2.16, 1.12)),
                rotation=(0, 0, 0)
            )

        return shapes.node


    # 在 rep.randomizer 中注册 get_shapes,是 rep.randomizer 可以直接调用get_shapes()函数
    rep.randomizer.register(get_shapes)
    # create the surface for the camera to focus on

    camera = rep.create.camera(position=sequential_pos[0], look_at=look_at_postion)
    render_product = rep.create.render_product(camera, resolution=(1280, 960))
    # trigger on frame for an interval
    with rep.trigger.on_frame(num_frames=100):
        # rep.randomizer.env_ckool_y(1)
        # rep.randomizer.env_ckool_r(1)
        rep.randomizer.env_ckool_p(1)
        rep.randomizer.env_chenpidan_r(1)
        # rep.randomizer.env_chenpidan_y(1)


        rep.randomizer.sphere_lights(10)
        rep.randomizer.get_shapes()
        with camera:
            rep.modify.pose(
                position=rep.distribution.uniform(sequential_pos[0],
                                                  sequential_pos[1]), look_at=look_at_postion
            )


    # 定义输出格式
    writer = rep.WriterRegistry.get("KittiWriter")
    writer.initialize(
        # 定义输出路径
        output_dir=
        "/home/zrt/xzc_code/Competition/AIRobot/image",
        # 定义边界框的最小阈值，物体在画面里的比例
        bbox_height_threshold=25,
        fully_visible_threshold=0.95,
        omit_semantic_type=True
    )
    writer.attach([render_product])
    rep.orchestrator.preview()