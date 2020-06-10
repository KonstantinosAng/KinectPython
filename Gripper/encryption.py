# Encrypt and Decrypt a string of data
import os
from cryptography.fernet import Fernet


# Generate a key to encrypt and decrypt messages
def generate_key():
    """
    Generates a key.key file with the encryption password inside the password_provided variable
    :return: None
    """
    # import here for optimization
    import base64
    from cryptography.hazmat.backends import default_backend
    from cryptography.hazmat.primitives import hashes
    from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC
    #### Change this to the password phrase you want (default root) #####
    password_provided = "root"  # This is input in the form of a string
    #####################################################################
    password = password_provided.encode()
    salt = b'salt_'  # CHANGE THIS - recommend using a key from os.urandom(16), must be of type bytes
    kdf = PBKDF2HMAC(
        algorithm=hashes.SHA256(),
        length=32,
        salt=salt,
        iterations=100000,
        backend=default_backend()
    )
    key = base64.urlsafe_b64encode(kdf.derive(password))  # Can only use kdf once
    file = open(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'key.key'), 'wb')
    file.write(key)  # The key is type bytes still
    file.close()


def encrypt(msg):
    """
    Encrypts a string of characters using the key.key encryption password
    :param msg: A string of characters
    :return Encrypted string of characters
    """
    if not os.path.exists((os.path.join(os.path.dirname(os.path.realpath(__file__)), 'key.key'))):
        print('Call generate_key() to generate an encryption key first')
    else:
        file = open(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'key.key'), 'rb')
        key = file.read()  # The key will be type bytes
        file.close()
        f = Fernet(key)
        return f.encrypt(msg)


def decrypt(msg):
    """
    Decrypts a string of characters using the key.key encryption password
    :param msg: An encrypted string of characters
    :return Decrypted string of characters
    """
    if not os.path.exists((os.path.join(os.path.dirname(os.path.realpath(__file__)), 'key.key'))):
        print('Call generate_key() to generate an encryption key first')
    else:
        file = open(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'key.key'), 'rb')
        key = file.read()  # The key will be type bytes
        file.close()
        f = Fernet(key)
        return f.decrypt(msg)


if __name__ == '__main__':
    # Test functionality
    generate_key()
    message = 'hello'
    one = encrypt(message.encode())
    two = decrypt(one)
    print(message)
    print(one)
    print(two)
