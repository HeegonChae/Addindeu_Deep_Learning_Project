import os
import random
from PIL import Image, ImageEnhance, ImageOps
import glob
from torchvision import transforms
import numpy as np

# 이미지 폴더 경로 설정
image_folder_path = 'diretory'

# 파일 경로에서 확장자 부분을 추출하는 함수
def get_extension(filename):
    parts = filename.split('.')
    return parts[-1] if len(parts) > 1 else ''

# 이미지 파일 리스트 가져오기 (JPG 확장자만 가져오기)
image_files = [f for f in glob.glob(os.path.join(image_folder_path, '*.*')) if get_extension(f).lower() == 'jpg']

# 데이터 증강 함수 정의
def augment_image(image):
    # Flip: Horizontal, Vertical
    if random.random() < 0.5:
        image = ImageOps.mirror(image)
    if random.random() < 0.5:
        image = ImageOps.flip(image)
    
    # 90° Rotate: Clockwise, Counter-Clockwise
    if random.random() < 0.5:
        image = image.rotate(90)
    if random.random() < 0.5:
        image = image.rotate(-90)
    
    # Crop: 0% Minimum Zoom, 20% Maximum Zoom
    if random.random() < 0.5:
        width, height = image.size
        crop_percent = random.uniform(0, 0.2)
        left = int(width * crop_percent)
        top = int(height * crop_percent)
        right = int(width * (1 - crop_percent))
        bottom = int(height * (1 - crop_percent))
        image = image.crop((left, top, right, bottom)).resize((width, height), Image.LANCZOS)
    
    # Rotation: Between -15° and +15°
    if random.random() < 0.5:
        angle = random.uniform(-15, 15)
        image = image.rotate(angle, resample=Image.BICUBIC, fillcolor="black")
    
    # Shear: ±15° Horizontal, ±15° Vertical
    if random.random() < 0.5:
        shear_x = random.uniform(-15, 15)
        shear_y = random.uniform(-15, 15)
        width, height = image.size
        xshift = abs(shear_x) * width / 100
        new_width = width + int(round(xshift))
        image = image.transform((new_width, height), Image.AFFINE, (1, shear_x / 100, -xshift if shear_x > 0 else 0, shear_y / 100, 1, 0), Image.BICUBIC)
    
    # Hue: Between -25° and +25°
    if random.random() < 0.5:
        enhancer = ImageEnhance.Color(image)
        hue_factor = random.uniform(-0.25, 0.25)
        image = enhancer.enhance(1 + hue_factor)
    
    return image

# 전체 이미지에 대해 증강 적용
for image_path in image_files:
    img = Image.open(image_path)
    img_augmented = augment_image(img)
    img_augmented.save(image_path)  # 원본 이미지 덮어쓰기

print(f"총 {len(image_files)}개의 이미지에 데이터 증강을 적용했습니다.")

