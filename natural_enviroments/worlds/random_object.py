import re
import random

def increment_tree_value(match):
    current_value = int(match.group(1))
    new_value = current_value + 1
    return f'tree_8_{new_value}'

def randomize_coordinates(match):
    x = round(random.uniform(-1000, 1000), 3)
    y = round(random.uniform(-1000, 1000), 3)
    return f'{x} {y} 40 0 -0 0'

# 초기 텍스트
input_text = """
<model name='tree_8_0'>
    <pose frame=''>-513.581 514.329 40 0 -0 0</pose>
    <scale>1 1 1</scale>
    <link name='link_0'>
        <pose frame=''>-513.581 514.329 40 1.49467 -0 0</pose>
        <velocity>0 0 0 0 -0 0</velocity>
        <acceleration>0 0 0 0 -0 0</acceleration>
        <wrench>0 0 0 0 -0 0</wrench>
    </link>
    <link name='link_1'>
        <pose frame=''>-513.581 514.329 40 1.57 -0 0</pose>
        <velocity>0 0 0 0 -0 0</velocity>
        <acceleration>0 0 0 0 -0 0</acceleration>
        <wrench>0 0 0 0 -0 0</wrench>
    </link>
</model>
"""

# 파일에 작성
output_file_path = '/home/kim/output.txt'
for i in range(1, 4):
    # tree_8_0 값을 1씩 증가하고 pose 값을 랜덤으로 변경
    output_text = re.sub(r"name='tree_8_(\d+)'", increment_tree_value, input_text)
    output_text = re.sub(r"<model name='tree_8_(\d+)'>", increment_tree_value, output_text)
    output_text = re.sub(r"<pose frame=''>-513.581 514.329 40 0 -0 0</pose>", randomize_coordinates, output_text)

    # 결과를 파일에 이어서 저장
    with open(output_file_path, 'a') as output_file:
        output_file.write(output_text)
        output_file.write("\n" + "="*40 + "\n")  # 구분을 위한 줄 구분

print(f"수정된 내용이 {output_file_path} 파일에 저장되었습니다.")

