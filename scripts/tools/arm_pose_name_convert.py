import sys

def format_conversion(original):
    formatted = original[0].lower()
    for char in original[1:]:
        if char.isupper():
            formatted += '_' + char.lower()
        else:
            formatted += char
    result = f"self.{formatted} = utilis.arm_pose()"
    return result

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python arm_pose_name_convert.py <OriginalName>")
    else:
        original = sys.argv[1]
        converted = format_conversion(original)
        print(converted)