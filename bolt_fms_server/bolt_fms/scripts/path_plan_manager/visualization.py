import matplotlib
matplotlib.use('TkAgg')  # GUI 백엔드 설정 (좌표 자동 표시 가능)

import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.image as mpimg
import matplotlib.font_manager as fm



def plot_multi_agent_paths(grid, agent_paths, background_path=None):
    rows, cols = len(grid), len(grid[0])
    colors = cm.get_cmap('tab10', len(agent_paths))

    # 배경 이미지 불러오기
    if background_path:
        img = mpimg.imread(background_path)
        img_height, img_width = img.shape[0], img.shape[1]
        fig, ax = plt.subplots(figsize=(cols, cols * img_height / img_width))
        ax.imshow(img, extent=[-0.5, cols - 0.5, rows - 0.5, -0.5], zorder=0)
        print(f"배경 이미지 크기: {img_width}x{img_height}", rows, cols)
        
    else:
        fig, ax = plt.subplots(figsize=(cols, rows))

    # 축 설정
    ax.set_xlim(-0.5, cols - 0.5)
    ax.set_ylim(rows - 0.5, -0.5)  # y축 반전
    ax.set_xticks(range(cols))
    ax.set_yticks(range(rows))
    ax.grid(True, color='gray', linestyle='--', linewidth=0.5, zorder=1)

    ax.set_title("Multi-Agent Time-Expanded A* Path Visualization", fontsize=14)

    # 장애물 표시 (테두리만 있는 사각형으로 변경)
    for y in range(rows):
        for x in range(cols):
            if grid[y][x] == 1:
                ax.add_patch(plt.Rectangle(
                    (x - 0.5, y - 0.5), 1, 1,
                    edgecolor='red',       # 테두리 색상
                    facecolor='none',      # 내부 채우기 없음
                    linewidth=1.5,         # 테두리 두께
                    zorder=2
                ))

    # 에이전트 경로 표시
    for idx, (agent_id, path) in enumerate(agent_paths.items()):
        color = colors(idx)
        positions = [pos for t, pos in path]
        xs, ys = zip(*positions)
        ax.plot(xs, ys, marker='o', label=agent_id, color=color, linewidth=2, zorder=3)
        ax.text(xs[0], ys[0], f"{agent_id}-S", color=color, fontsize=9, ha='right', va='bottom', zorder=4)
        ax.text(xs[-1], ys[-1], f"{agent_id}-G", color=color, fontsize=9, ha='left', va='top', zorder=4)

    ax.legend(loc='lower right')
    plt.tight_layout()
    plt.show()
