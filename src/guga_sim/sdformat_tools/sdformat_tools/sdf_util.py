import os

from ament_index_python.packages import get_packages_with_prefixes

MODEL_URI_PREFIX = 'model://'

sdf_paths = []
if os.getenv("IGN_GAZEBO_RESOURCE_PATH") is not None:
    sdf_paths = sdf_paths+os.getenv("IGN_GAZEBO_RESOURCE_PATH").split(":")
if os.getenv("GAZEBO_MODEL_PATH") is not None:
    sdf_paths = sdf_paths+os.getenv("GAZEBO_MODEL_PATH").split(":")

def get_model_directory(model_name):
    for tmp_dir in sdf_paths:
        model_directory = os.path.join(tmp_dir, model_name)
        if(os.path.isdir(model_directory)):
            return model_directory
    return ""

# get absolute path according to uri
def parse_model_uri(uri):
    if uri.find(MODEL_URI_PREFIX) != 0:
        return ''
    tmp_uri = uri[len(MODEL_URI_PREFIX):]
    pos = tmp_uri.find('/')
    if pos == -1:
        return ''
    model_name = tmp_uri[0:pos]
    model_dir_path = get_model_directory(model_name)
    if model_dir_path == '':
        return ''
    return model_dir_path + tmp_uri[pos:]


def model_uri_to_urdf_uri(uri):
    """Resolve a Gazebo model URI to a relocatable URI understood by URDF."""
    model_path = parse_model_uri(uri)
    if model_path == '':
        return ''

    model_path = os.path.abspath(model_path)
    for package_name, prefix in get_packages_with_prefixes().items():
        package_share = os.path.abspath(
            os.path.join(prefix, 'share', package_name)
        )
        try:
            if os.path.commonpath((model_path, package_share)) != package_share:
                continue
        except ValueError:
            continue

        relative_path = os.path.relpath(model_path, package_share)
        return 'package://{}/{}'.format(package_name, relative_path)

    return 'file://' + model_path
